#!/usr/bin/env python3
"""Compile the actual native start helper and inject transport failures."""
import argparse,pathlib,subprocess,tempfile
p=argparse.ArgumentParser();p.add_argument('--source',type=pathlib.Path);args=p.parse_args()
source=args.source or pathlib.Path(__file__).resolve().parents[4]/'drivers/media/i2c/imx415.c'
text=source.read_text();start=text.index('static int imx415_native_fhd_start(')
brace=text.index('{',start);depth=1;end=brace+1
while depth:
 depth+=(text[end]=='{')-(text[end]=='}');end+=1
helper=text[start:end]
harness=r'''
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#define IMX415_FHD_UNCERTAIN 4
#define IMX415_FHD_RELEASED 3
struct imx415 {void *client;unsigned native_fhd_stage;};
static int calls,fail_at,delay_seen,writes;
static unsigned write_addr[3],write_value[3];
static void usleep_range(unsigned low,unsigned high) {assert(low>=80000 && high>=low);delay_seen=1;}
static int imx415_native_fhd_check_reg(struct imx415 *s,unsigned addr,unsigned value) {
 assert(s && delay_seen);assert(addr==0x3000 || addr==0x3002);assert(value<=1);
 return ++calls==fail_at ? -EIO : 0;
}
static int imx415_write_reg(void *client,unsigned addr,unsigned size,unsigned value) {
 (void)client;assert(delay_seen && size==1);assert(writes<3);
 write_addr[writes]=addr;write_value[writes++]=value;
 return ++calls==fail_at ? -EIO : 0;
}
'''
cases=r'''
int main(void) {
 for(int failure=0;failure<=4;failure++) {
  struct imx415 s={0};calls=0;fail_at=failure;delay_seen=0;writes=0;
  int ret=imx415_native_fhd_start(&s);
  if(!failure) {
   assert(ret==0 && s.native_fhd_stage==IMX415_FHD_RELEASED && writes==1);
   assert(write_addr[0]==0x3002 && write_value[0]==0);
  } else {
   assert(ret==-EIO && s.native_fhd_stage==IMX415_FHD_UNCERTAIN);
   assert(writes>=2 && write_addr[writes-2]==0x3002 && write_value[writes-2]==1);
   assert(write_addr[writes-1]==0x3000 && write_value[writes-1]==1);
  }
 }
 puts("IMX415_NATIVE_START_PASS: delayed release/readback and all four I2C failure edges");
}
'''
with tempfile.TemporaryDirectory(prefix='imx415-start-') as td:
 c=pathlib.Path(td)/'test.c';exe=pathlib.Path(td)/'test';c.write_text(harness+helper+cases)
 subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror',str(c),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
