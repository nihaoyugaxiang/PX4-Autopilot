#!/usr/bin/env python3
"""Compile production UART lifecycle against separate CLI/worker fd tables.
The syscall model checks ownership; it does not emulate NuttX or pin waveforms.
"""
from pathlib import Path
import subprocess
import tempfile
import sys
ROOT = Path(__file__).resolve().parents[2]
source = Path(sys.argv[1]).read_text() if len(sys.argv) > 1 else (ROOT/'src/modules/vgsolar_link/VgsolarLink.cpp').read_text()
def extract(signature):
    start = source.index(signature)
    opening = source.index('{', start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]
spawn = extract('static int task_spawn(')
run = extract('void Run() override')
run = run[:run.index('\t\tconst uint64_t now =')] + '''
        assert(_fd >= 0 && owner_group == current_group);
        ++io_calls;
    }
'''
harness = r'''
#include <atomic>
#include <cassert>
#include <cstdint>
constexpr int PX4_ERROR=-1, PX4_OK=0, task_id_is_work_queue=-2;
static int current_group=1, owner_group=0, opens=0, closes=0, io_calls=0;
static bool fail_open=false;
struct Base { virtual void Run()=0; virtual ~Base()=default; };
class VgsolarLink: public Base {
public:
    static std::atomic<VgsolarLink *> _object;
    static int _task_id;
    int _fd=-1;
    bool exiting=false, cleaned=false, scheduled=false;
    bool should_exit(){return exiting;}
    void ScheduleOnInterval(int interval){assert(interval==10000);scheduled=true;}
    void ScheduleClear(){scheduled=false;}
    void exit_and_cleanup(){cleaned=true;}
    bool open_uart(){++opens;owner_group=current_group;if(fail_open)return false;_fd=7;return true;}
    void close_uart(){if(_fd>=0){assert(owner_group==current_group);++closes;_fd=-1;}}
''' + spawn + '\n' + run + r'''
};
std::atomic<VgsolarLink *> VgsolarLink::_object{nullptr};
int VgsolarLink::_task_id=-1;
static VgsolarLink *start(){
    current_group=1;
    const int before=opens;
    assert(VgsolarLink::task_spawn(0,nullptr)==PX4_OK);
    assert(opens==before);
    auto *link=VgsolarLink::_object.load();assert(link && link->scheduled);
    return link;
}
int main(){
    auto *link=start();
    current_group=2;link->Run();assert(opens==1 && io_calls==1 && owner_group==2);
    link->Run();assert(opens==1 && io_calls==2);
    link->exiting=true;link->Run();assert(closes==1 && link->cleaned && !link->scheduled);delete link;
    link=start();current_group=2;link->exiting=true;link->Run();
    assert(opens==1 && closes==1 && link->cleaned);delete link;
    link=start();current_group=2;fail_open=true;link->Run();
    assert(opens==2 && io_calls==2 && closes==1 && link->cleaned && !link->scheduled);delete link;
}
'''
with tempfile.TemporaryDirectory(prefix='vgsolar-uart-owner-') as directory:
    temp=Path(directory)
    (temp/'check.cpp').write_text(harness)
    subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',str(temp/'check.cpp'),'-o',str(temp/'check')],check=True)
    result=subprocess.run([str(temp/'check')],capture_output=True,text=True)
    if result.returncode:
        print(result.stderr, file=sys.stderr)
        raise SystemExit(1)
print('PASS: production NCU lifecycle opens/uses/closes UART in work queue; stop-before-open and open failure clean up')
