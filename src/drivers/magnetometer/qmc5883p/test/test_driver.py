"""Compile the actual driver against fake I2C, including a held reset bit.

No board is accessed. --revision tests older production code against this model.
QST Rev E supplies the map/scales. The held-reset case covers the B2 board
observation; reset assertion/deassertion also follows the Linux driver author's
hardware-tested sequence, https://lists.openwall.net/linux-kernel/2026/04/09/1855.
This is a fault-injection model, not proof of real sensor timing or heading.
"""
import argparse
import pathlib
import subprocess
import tempfile

STUBS = r'''
#pragma once
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
#include <utility>
#define PX4_OK 0
#define PX4_ERROR -1
inline void test_log(const char *, ...) {}
#define PX4_INFO(...) test_log(__VA_ARGS__)
#define PX4_DEBUG(...) test_log(__VA_ARGS__)
#define DEVICE_DEBUG(...) test_log(__VA_ARGS__)
#define MODULE_NAME "qmc5883p"
namespace time_literals {
constexpr uint64_t operator""_ms(unsigned long long v) {return v*1000;}
constexpr uint64_t operator""_s(unsigned long long v) {return v*1000000;}
}
using hrt_abstime = uint64_t;
inline uint64_t test_time{};
inline uint64_t hrt_absolute_time() {return test_time;}
inline uint64_t hrt_elapsed_time(const uint64_t *v) {return test_time-*v;}
struct Perf {const char *name; unsigned count{};};
using perf_counter_t = Perf*;
inline unsigned transfer_errors{};
constexpr int PC_COUNT=0;
inline perf_counter_t perf_alloc(int, const char *name) {return new Perf{name,0};}
inline void perf_count(perf_counter_t v) {
    ++v->count;
    if (std::strstr(v->name,"bad transfer")) ++transfer_errors;
}
inline void perf_free(perf_counter_t v) {delete v;}
inline void perf_print_counter(perf_counter_t) {}
struct I2CSPIDriverConfig {int rotation{};};
struct Write {uint8_t reg; uint8_t value; uint64_t time;};
struct FakeChip {
    std::array<uint8_t,256> regs{};
    std::vector<Write> writes;
    std::vector<std::array<float,3>> publications;
    bool auto_clear_reset{};
    bool reject_assert{}, reject_release{}, reject_sign_write{}, reject_range_write{};
    bool reject_ctrl2_read{}, reject_sample{}, hold_reset{};
    unsigned reset_count{};
    uint64_t asserted_at{}, sample_at{}, next_run{}, interval{};
    bool scheduled{};
    FakeChip() {regs[0]=0x80;}
    void assert_reset() {
        regs.fill(0);
        regs[0]=0x80;
        regs[0x0b]=auto_clear_reset ? 0 : 0x80;
        asserted_at=test_time;
        sample_at=test_time+1000;
        ++reset_count;
    }
    void measurement() {
        if ((regs[0x0b]&0x80) || (regs[0x0a]&3)!=1 || test_time<sample_at) return;
        const int16_t xyz[]{3750,7500,-3750};
        for(unsigned i=0;i<3;++i) {
            regs[1+2*i]=uint8_t(xyz[i]);
            regs[2+2*i]=uint8_t(uint16_t(xyz[i])>>8);
        }
        regs[9]=1;
        sample_at=test_time+20000;
    }
};
inline FakeChip chip;
namespace device {
class I2C {
public:
    explicit I2C(const I2CSPIDriverConfig&) {}
    virtual ~I2C()=default;
    virtual int init() {return probe();}
    virtual int probe()=0;
    unsigned get_device_id() const {return 994337;}
    int transfer(const uint8_t *tx, unsigned nt, uint8_t *rx, unsigned nr) {
        assert(nt>0);
        if(nt==2 && nr==0) {
            const uint8_t reg=tx[0], val=tx[1];
            if (reg==0x0b && (val&0x80) && chip.reject_assert) return -1;
            if (reg==0x0b && val==0 && chip.reject_release) return -1;
            if (reg==0x0b && val==8 && chip.reject_range_write) return -1;
            if (reg==0x29 && chip.reject_sign_write) return -1;
            chip.writes.push_back({reg,val,test_time});
            if (reg==0x0b && (val&0x80)) {
                chip.assert_reset();
            } else if (reg==0x0b && val==0) {
                if (!chip.hold_reset) chip.regs[reg]=val;
            } else if (!(chip.regs[0x0b]&0x80)) {
                chip.regs[reg]=val;
            }
            return 0;
        }
        assert(nt==1);
        if (tx[0]==0x0b && chip.reject_ctrl2_read) return -1;
        if (tx[0]==1 && chip.reject_sample) return -1;
        for(unsigned i=0;i<nr;++i) {
            const unsigned reg=tx[0]+i;
            rx[i]=chip.regs[reg];
            // STATUS clears DRDY/OVL, never the soft-reset bit in CNTL2.
            if (reg==9) chip.regs[reg]=0;
        }
        return 0;
    }
protected:
    int _retries{};
};
}
struct I2CSPIDriverBase {virtual void print_status() {}};
template<class T> struct I2CSPIDriver : I2CSPIDriverBase {
    explicit I2CSPIDriver(const I2CSPIDriverConfig&) {}
    void ScheduleClear() {chip.scheduled=false; chip.interval=0;}
    void ScheduleNow() {chip.scheduled=true; chip.next_run=test_time;}
    void ScheduleDelayed(uint64_t t) {chip.scheduled=true; chip.interval=0; chip.next_run=test_time+t;}
    void ScheduleOnInterval(uint64_t t,uint64_t delay) {
        chip.scheduled=true; chip.interval=t; chip.next_run=test_time+delay;
    }
};
class PX4Magnetometer {
    float scale=1;
public:
    PX4Magnetometer(unsigned,int) {}
    void set_scale(float value) {scale=value;}
    void update(uint64_t,int16_t x,int16_t y,int16_t z) {
        assert(!(chip.regs[0x0b]&0x80));
        chip.publications.push_back({x*scale,y*scale,z*scale});
    }
};
'''

MAIN = r'''
#include "QMC5883P.hpp"
static void require(bool ok,const char *what) {
    if(!ok) {std::fprintf(stderr,"FAIL: %s\n",what); std::exit(1);}
}
static void ticks(QMC5883P &d,unsigned count) {
    for(unsigned i=0;i<count;++i) {
        require(chip.scheduled,"driver must remain scheduled");
        test_time=chip.next_run;
        if (chip.interval) chip.next_run+=chip.interval;
        else chip.scheduled=false;
        chip.measurement();
        d.RunImpl();
    }
}
static void fresh() {chip=FakeChip{}; test_time=0; transfer_errors=0;}
static unsigned find_write(uint8_t reg,uint8_t value,unsigned begin=0) {
    for(unsigned i=begin;i<chip.writes.size();++i)
        if(chip.writes[i].reg==reg && chip.writes[i].value==value) return i;
    return 99999;
}
static void initialized(const char *what) {
    require(chip.regs[0x0a]==0xc5 && chip.regs[0x0b]==8 && !chip.publications.empty(),what);
}
int main() {
    for (const bool self_clear : {false,true}) {
        fresh(); chip.auto_clear_reset=self_clear;
        I2CSPIDriverConfig cfg; QMC5883P d(cfg);
        require(d.init()==0,"probe/init");
        ticks(d,1);
        require(chip.writes[0].reg==0x0b && chip.writes[0].value==0x80,"assert real reset bit");
        if (!self_clear) {
            uint8_t cmd=9, status{};
            require(d.transfer(&cmd,1,&status,1)==0 && chip.regs[0x0b]==0x80,"reading STATUS must not release reset");
        }
        require(chip.publications.empty(),"no sample during reset");
        ticks(d,6);
        const unsigned release=find_write(0x0b,0), sign=find_write(0x29,6);
        const unsigned range=find_write(0x0b,8), enable=find_write(0x0a,0xc5);
        require(release<sign && sign<range && range<enable,"release reset, vendor setup, range, enable");
        require(chip.writes[release].time-chip.writes[0].time>=1000,"reset settling before deassertion");
        initialized("held/self-clearing reset initialization and publication");
        const auto v=chip.publications.back();
        require(std::fabs(v[0]+2)<1e-5 && std::fabs(v[1]+1)<1e-5 && std::fabs(v[2]-1)<1e-5,"Gauss scaling and existing axes");
        const unsigned resets=chip.reset_count;
        ticks(d,200);
        require(chip.reset_count==resets && chip.publications.size()>190,"sustained samples without reset loop");
        const unsigned before=chip.writes.size();
        chip.regs[0x0b]=0x0f;
        ticks(d,16);
        require(find_write(0x0b,0x80,before)<99999,"corrupt register forces reset");
        initialized("corrupt register recovery");
        std::printf("PASS: %s reset, scheduling, sustained data, scale/axes, corrupt-register recovery\n",self_clear?"self-clearing":"held");
    }
    for (unsigned fault=0;fault<6;++fault) {
        fresh();
        if(fault==0) chip.reject_assert=true;
        if(fault==1) chip.reject_release=true;
        if(fault==2) chip.reject_ctrl2_read=true;
        if(fault==3) chip.hold_reset=true;
        if(fault==4) chip.reject_sign_write=true;
        if(fault==5) chip.reject_range_write=true;
        I2CSPIDriverConfig cfg; QMC5883P d(cfg);
        require(d.init()==0,"fault test probe");
        ticks(d,140);
        require(chip.publications.empty(),"initialization failure must not publish");
        require(chip.regs[0x0a]==0,"initialization failure must not enable acquisition");
        if (fault!=3) require(transfer_errors>0,"initialization transfer errors must be counted");
        chip.reject_assert=chip.reject_release=chip.reject_ctrl2_read=false;
        chip.hold_reset=chip.reject_sign_write=chip.reject_range_write=false;
        ticks(d,14);
        initialized("initialization fault recovery");
        std::printf("PASS: init fault %u blocked data and recovered\n",fault);
    }
    {
        fresh(); I2CSPIDriverConfig cfg; QMC5883P d(cfg);
        require(d.init()==0,"sample fault probe"); ticks(d,7);
        const auto count=chip.publications.size();
        chip.reject_sample=true; ticks(d,14);
        require(chip.publications.size()==count && transfer_errors>=11,"sample transfer failures stop data and trigger recovery");
        chip.reject_sample=false; ticks(d,14);
        require(chip.publications.size()>count,"sample transfer recovery");
        std::puts("PASS: sample NACK diagnostics and recovery");
    }
}
'''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=pathlib.Path, default=pathlib.Path.cwd())
    parser.add_argument('--revision')
    args = parser.parse_args()
    rel = pathlib.Path('src/drivers/magnetometer/qmc5883p')
    with tempfile.TemporaryDirectory(prefix='vgsolar-qmc-test-') as folder:
        temp = pathlib.Path(folder)
        for name in ('QMC5883P.cpp', 'QMC5883P.hpp', 'QST_QMC5883P_registers.hpp'):
            if args.revision:
                source = subprocess.check_output(['git', 'show', f'{args.revision}:{rel/name}'], cwd=args.root, text=True)
            else:
                source = (args.root/rel/name).read_text()
            if name == 'QMC5883P.hpp':
                source = '\n'.join(line for line in source.splitlines() if not line.startswith('#include <'))
                source = '#include "stubs.hpp"\n' + source
            (temp/name).write_text(source)
        (temp/'stubs.hpp').write_text(STUBS)
        (temp/'test.cpp').write_text('#include <cstdlib>\n' + MAIN)
        executable = temp/'test'
        subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                        str(temp/'QMC5883P.cpp'), str(temp/'test.cpp'), '-o', str(executable)], check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == '__main__':
    main()
