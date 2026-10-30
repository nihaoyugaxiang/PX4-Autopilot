"""Run the actual QMC5883P driver against a small fake I2C device.

No board is accessed. Pass --revision <git-ref> to check an earlier driver.
The oracle values follow QST QMC5883P Rev E sections 2, 7 and 9.
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
#define PX4_INFO(...) ((void)0)
#define PX4_DEBUG(...) ((void)0)
#define DEVICE_DEBUG(...) ((void)0)
#define MODULE_NAME "qmc5883p"
namespace time_literals {
constexpr uint64_t operator""_ms(unsigned long long v) {return v*1000;}
constexpr uint64_t operator""_s(unsigned long long v) {return v*1000000;}
}
using hrt_abstime = uint64_t;
inline uint64_t test_time{};
inline uint64_t hrt_absolute_time() {return test_time;}
inline uint64_t hrt_elapsed_time(const uint64_t *v) {return test_time-*v;}
using perf_counter_t = unsigned*;
constexpr int PC_COUNT=0;
inline perf_counter_t perf_alloc(int, const char*) {return new unsigned{};}
inline void perf_count(perf_counter_t v) {++*v;}
inline void perf_free(perf_counter_t v) {delete v;}
inline void perf_print_counter(perf_counter_t) {}
struct I2CSPIDriverConfig {};
struct FakeChip {
    std::array<uint8_t,256> regs{};
    std::vector<std::pair<uint8_t,uint8_t>> writes;
    std::vector<std::array<float,3>> publications;
    bool reject_sign_write{};
    FakeChip() {reset();}
    void reset() {regs.fill(0); regs[0]=0x80;}
    void measurement() {
        const int16_t xyz[]{3750,7500,-3750};
        for(unsigned i=0;i<3;++i) {
            regs[1+2*i]=uint8_t(xyz[i]);
            regs[2+2*i]=uint8_t(uint16_t(xyz[i])>>8);
        }
        regs[9]=1;
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
            if(tx[0]==0x29 && chip.reject_sign_write) return -1;
            chip.writes.emplace_back(tx[0],tx[1]);
            if(tx[0]==0x0b && (tx[1]&0x80)) chip.reset();
            else chip.regs[tx[0]]=tx[1];
            return 0;
        }
        assert(nt==1);
        for(unsigned i=0;i<nr;++i) rx[i]=chip.regs[tx[0]+i];
        return 0;
    }
protected:
    int _retries{};
};
}
struct I2CSPIDriverBase {virtual void print_status() {}};
template<class T> struct I2CSPIDriver : I2CSPIDriverBase {
    explicit I2CSPIDriver(const I2CSPIDriverConfig&) {}
    void ScheduleClear() {}
    void ScheduleNow() {}
    void ScheduleDelayed(uint64_t) {}
    void ScheduleOnInterval(uint64_t,uint64_t) {}
};
class PX4Magnetometer {
    float scale=1;
public:
    PX4Magnetometer(unsigned,int) {}
    void set_scale(float value) {scale=value;}
    void update(uint64_t,int16_t x,int16_t y,int16_t z) {
        chip.publications.push_back({x*scale,y*scale,z*scale});
    }
};
'''.replace('struct I2CSPIDriverConfig {};', 'struct I2CSPIDriverConfig {int rotation{};};')

MAIN = r'''
#include "QMC5883P.hpp"
static void require(bool ok,const char *what) {
    if(!ok) {std::fprintf(stderr,"FAIL: %s\n",what); std::exit(1);}
}
static void ticks(QMC5883P &d,unsigned count) {
    for(unsigned i=0;i<count;++i) {
        test_time+=100000;
        chip.measurement();
        d.RunImpl();
    }
}
int main() {
    {
        chip=FakeChip{};
        I2CSPIDriverConfig cfg;
        QMC5883P d(cfg);
        require(d.init()==0,"probe/init");
        ticks(d,5);
        require(!chip.writes.empty() && chip.writes.front()==std::make_pair(uint8_t(0x0b),uint8_t(0x80)),"real soft reset bit");
        unsigned sign=999,range=999,enable=999;
        for(unsigned i=0;i<chip.writes.size();++i) {
            const auto [reg,val]=chip.writes[i];
            if(reg==0x29 && val==6) sign=i;
            if(reg==0x0b && val==8) range=i;
            if(reg==0x0a && val==0xc5) enable=i;
        }
        require(sign<range && range<enable,"vendor setup, range, then normal-mode enable");
        require(chip.regs[0x0a]==0xc5 && chip.regs[0x0b]==8,"50 Hz normal mode, OSR1/2=8, auto set/reset, +/-8 G");
        require(!chip.publications.empty(),"data publication");
        const auto v=chip.publications.back();
        require(std::fabs(v[0]+2)<1e-5 && std::fabs(v[1]+1)<1e-5 && std::fabs(v[2]-1)<1e-5,"Gauss conversion and existing axis mapping");
        const auto before=chip.writes.size();
        chip.regs[0x0b]=0x0f;
        ticks(d,8);
        bool recovered=false;
        for(unsigned i=before;i<chip.writes.size();++i)
            if(chip.writes[i]==std::make_pair(uint8_t(0x0b),uint8_t(0x80))) recovered=true;
        require(recovered && chip.regs[0x0b]==8,"register corruption triggers reset/reconfiguration");
        std::puts("PASS: initialization, scaling/axes, register corruption recovery");
    }
    {
        chip=FakeChip{};
        chip.reject_sign_write=true;
        I2CSPIDriverConfig cfg;
        QMC5883P d(cfg);
        require(d.init()==0,"probe for setup-write failure test");
        ticks(d,5);
        require(chip.publications.empty(),"failed required vendor write must prevent publication");
        chip.reject_sign_write=false;
        ticks(d,6);
        require(!chip.publications.empty(),"setup retries recover after I2C failure");
        std::puts("PASS: required-write NACK gating and recovery");
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
                text = subprocess.check_output(['git', 'show', f'{args.revision}:{rel/name}'], cwd=args.root, text=True)
            else:
                text = (args.root/rel/name).read_text()
            if name == 'QMC5883P.hpp':
                text = '\n'.join(line for line in text.splitlines() if not line.startswith('#include <'))
                text = '#include "stubs.hpp"\n' + text
            (temp/name).write_text(text)
        (temp/'stubs.hpp').write_text(STUBS)
        (temp/'test.cpp').write_text('#include <cstdlib>\n' + MAIN)
        executable = temp/'test'
        subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                        str(temp/'QMC5883P.cpp'), str(temp/'test.cpp'), '-o', str(executable)], check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == '__main__':
    main()
