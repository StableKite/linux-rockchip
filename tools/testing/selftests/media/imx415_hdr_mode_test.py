#!/usr/bin/env python3
"""Execute the driver's HDR selector against colliding real mode geometries."""
import argparse,pathlib,subprocess,tempfile
p=argparse.ArgumentParser();p.add_argument('--source',type=pathlib.Path);args=p.parse_args()
source=args.source or pathlib.Path(__file__).resolve().parents[4]/'drivers/media/i2c/imx415.c'
text=source.read_text();start=text.index('static const struct imx415_mode *imx415_find_hdr_mode(')
brace=text.index('{',start);depth=1;end=brace+1
while depth:
    depth+=(text[end]=='{')-(text[end]=='}');end+=1
helper=text[start:end]
harness=r'''
#include <assert.h>
#include <stddef.h>
#include <stdio.h>
typedef unsigned int u32;
struct imx415_mode {u32 width,height,bus_fmt,hdr_mode;};
struct imx415 {const struct imx415_mode *cur_mode,*supported_modes;u32 cfg_num;};
'''
cases=r'''
int main(void) {
 const struct imx415_mode modes[]={
  {1944,1097,12,0}, {3864,2192,10,0}, {3864,2192,12,1},
  {3864,2192,10,1}, {3864,2192,10,0}, {1944,1097,10,0}};
 struct imx415 s={&modes[5],modes,6};
 assert(imx415_find_hdr_mode(&s,0)==&modes[5]); /* native RAW10, not legacy RAW12 */
 s.cur_mode=&modes[0];assert(imx415_find_hdr_mode(&s,0)==&modes[0]);
 s.cur_mode=&modes[4];assert(imx415_find_hdr_mode(&s,0)==&modes[4]); /* preserve FPS variant */
 assert(imx415_find_hdr_mode(&s,1)==&modes[3]); /* preserve RAW10 when changing HDR */
 s.cur_mode=&modes[2];assert(imx415_find_hdr_mode(&s,0)==NULL); /* no RAW12 4K linear */
 s.cur_mode=&modes[5];assert(imx415_find_hdr_mode(&s,1)==NULL); /* no native FHD HDR */
 puts("IMX415_HDR_MODE_PASS: native/legacy/FPS retention, bit-depth-safe HDR and absent modes");
}
'''
with tempfile.TemporaryDirectory(prefix='imx415-hdr-') as td:
 c=pathlib.Path(td)/'test.c';exe=pathlib.Path(td)/'test'
 c.write_text(harness+helper+cases)
 subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror',str(c),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
