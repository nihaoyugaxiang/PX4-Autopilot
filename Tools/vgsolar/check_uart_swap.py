#!/usr/bin/env python3
"""Exercise production NCU UART open/close with host PTY and controlled SWAP ioctl."""
from pathlib import Path
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[2]
source = (ROOT/'src/modules/vgsolar_link/VgsolarLink.cpp').read_text()
def extract(signature):
    start=source.index(signature)
    opening=source.index('{',start)
    depth=1; end=opening+1
    while depth:
        depth+=(source[end]=='{')-(source[end]=='}'); end+=1
    return source[start:end]
methods=extract('bool open_uart()')+'\n'+extract('void close_uart()')
harness=r'''
#include <cassert>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#define PX4_ERR(...) ((void)0)
#define PX4_INFO(...) ((void)0)
constexpr int PX4_OK=0, PARAM_INVALID=-1;
using param_t=int;
static int32_t configured_swap;
static bool missing=false, get_error=false, ioctl_error=false;
static int ioctl_calls=0, last_arg=-1;
static param_t param_find(const char *name) {
 assert(std::strcmp(name,"VGS_CC_SWAP")==0); return missing ? PARAM_INVALID : 1;
}
static int param_get(param_t handle, int32_t *value) {
 assert(handle==1); *value=configured_swap; return get_error ? -1 : 0;
}
#ifdef CONFIG_STM32H7_USART_SWAP
constexpr int TIOCSSWAP=0x35, SER_SWAP_ENABLED=1;
static int test_ioctl(int fd, int operation, int argument) {
 assert(fd>=0 && operation==TIOCSSWAP);
 ++ioctl_calls; last_arg=argument;
 return ioctl_error ? -1 : 0;
}
#define ioctl test_ioctl
#endif
class Link {
public:
 const char *SERIAL_PATH;
 int _fd=-1;
 bool _swap_enabled=false;
 std::atomic<bool> _active{false};
 explicit Link(const char *path):SERIAL_PATH(path){}
 ~Link(){close_uart();}
''' + methods + r'''
};
int main() {
 int master=posix_openpt(O_RDWR|O_NOCTTY);
 assert(master>=0 && grantpt(master)==0 && unlockpt(master)==0);
 const char *path=ptsname(master); assert(path);
 Link link(path);
 configured_swap=0;
 assert(link.open_uart() && link._active && !link._swap_enabled);
#ifdef CONFIG_STM32H7_USART_SWAP
 assert(ioctl_calls==1 && last_arg==0);
#endif
 link.close_uart(); assert(link._fd==-1 && !link._active);
 configured_swap=1;
#ifdef CONFIG_STM32H7_USART_SWAP
 assert(link.open_uart() && link._active && link._swap_enabled && last_arg==1);
 link.close_uart();
 // Restoring zero must explicitly clear a swap that survives a module restart.
 configured_swap=0;
 assert(link.open_uart() && !link._swap_enabled && last_arg==0);
 link.close_uart();
 ioctl_error=true;
 assert(!link.open_uart() && link._fd==-1 && !link._active);
 ioctl_error=false;
#else
 assert(!link.open_uart() && link._fd==-1 && !link._active);
 assert(ioctl_calls==0 && last_arg==-1 && !ioctl_error);
#endif
 configured_swap=2;
 assert(!link.open_uart() && link._fd==-1 && !link._active);
 configured_swap=0; missing=true;
 assert(!link.open_uart() && link._fd==-1 && !link._active);
 missing=false; get_error=true;
 assert(!link.open_uart() && link._fd==-1 && !link._active);
 get_error=false;
 Link bad("/no/such/serial/device");
 assert(!bad.open_uart() && bad._fd==-1 && !bad._active);
 close(master);
}
'''
with tempfile.TemporaryDirectory(prefix='vgsolar-uart-swap-') as temp:
    temp=Path(temp); (temp/'check.cpp').write_text(harness)
    for supported in (False,True):
        flags=['-D__PX4_NUTTX','-DCONFIG_STM32H7_USART_SWAP'] if supported else []
        subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',
                        *flags,str(temp/'check.cpp'),'-o',str(temp/'check')],check=True)
        subprocess.run([str(temp/'check')],check=True)
print('PASS: production NCU UART startup handles swap on/off/restore, unsupported driver, bad parameter, ioctl failure and cleanup using host PTY')
