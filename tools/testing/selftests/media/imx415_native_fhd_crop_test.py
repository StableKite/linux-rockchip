#!/usr/bin/env python3
"""Compile and execute the real selection callback without touching hardware."""
import argparse,pathlib,re,subprocess,tempfile
p=argparse.ArgumentParser();p.add_argument('--source',type=pathlib.Path)
args=p.parse_args()
source=args.source or pathlib.Path(__file__).resolve().parents[4]/'drivers/media/i2c/imx415.c'
text=source.read_text()
start=text.index('static int imx415_get_selection(')
brace=text.index('{',start);depth=1;end=brace+1
while depth:
    depth+=(text[end]=='{')-(text[end]=='}');end+=1
callback=text[start:end]
defines='\n'.join(x for x in text.splitlines() if x.startswith(('#define CROP_START','#define DST_WIDTH_','#define DST_HEIGHT_')))
harness=r'''
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <stdio.h>
#define V4L2_SEL_TGT_CROP_BOUNDS 2
struct mode {unsigned width,height;bool fullhd_replace_init;};
struct imx415 {struct mode *cur_mode;};
struct v4l2_subdev {struct imx415 *sensor;};
struct v4l2_subdev_state {int unused;};
struct v4l2_subdev_selection {unsigned target;struct {unsigned left,top,width,height;}r;};
#define to_imx415(sd) ((sd)->sensor)
'''
cases=r'''
static void check(unsigned w,unsigned h,bool native,unsigned x,unsigned y,unsigned ow,unsigned oh) {
 struct mode m={w,h,native};struct imx415 sensor={&m};struct v4l2_subdev sd={&sensor};
 struct v4l2_subdev_selection selection={.target=V4L2_SEL_TGT_CROP_BOUNDS};
 assert(imx415_get_selection(&sd,NULL,&selection)==0);
 assert(selection.r.left==x && selection.r.top==y && selection.r.width==ow && selection.r.height==oh);
 selection.target=999;assert(imx415_get_selection(&sd,NULL,&selection)==-EINVAL);
}
int main(void) {
 check(1944,1097,true,4,0,1936,1096);
 check(1944,1097,false,12,8,1920,1080);
 check(3864,2192,false,12,16,3840,2160);
 check(1280,720,false,0,0,1280,720);
 puts("IMX415_NATIVE_FHD_CROP_PASS: native, legacy FHD, legacy 4K, fallback and invalid target");
}
'''
with tempfile.TemporaryDirectory(prefix='imx415-crop-') as td:
    c=pathlib.Path(td)/'test.c';exe=pathlib.Path(td)/'test'
    c.write_text(harness+'\n'+defines+'\n'+callback+'\n'+cases)
    subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-Wno-unused-parameter',str(c),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
