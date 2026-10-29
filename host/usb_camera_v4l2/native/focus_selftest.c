// Driver-boundary checks against the actual capture.c focus functions.
// No camera or /dev node is opened by this executable.
#define _POSIX_C_SOURCE 200809L
#include <time.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
static int automatic,position,step,maximum,flags,ignore_auto,ignore_position;
static unsigned query_fail,set_fail,get_fail;
static int writes[4],write_count;
static int fake_ioctl(int fd,unsigned long op,...){
 (void)fd;va_list args;va_start(args,op);void *p=va_arg(args,void *);va_end(args);
 if(op==VIDIOC_QUERYCTRL){
  struct v4l2_queryctrl *q=p;unsigned id=q->id;
  if(id==query_fail){errno=EINVAL;return -1;}
  memset(q,0,sizeof(*q));q->id=id;q->minimum=0;q->flags=(unsigned)flags;
  q->maximum=id==V4L2_CID_FOCUS_AUTO?1:maximum;q->step=id==V4L2_CID_FOCUS_AUTO?1:step;
  return 0;
 }
 struct v4l2_control *c=p;
 if(op==VIDIOC_S_CTRL){
  if(c->id==set_fail){errno=EIO;return -1;}
  if(write_count<4)writes[write_count++]=c->id==V4L2_CID_FOCUS_AUTO?1:2;
  if(c->id==V4L2_CID_FOCUS_AUTO){if(!ignore_auto)automatic=c->value;}
  else if(!ignore_position)position=c->value;
  return 0;
 }
 if(op==VIDIOC_G_CTRL){
  if(c->id==get_fail){errno=EIO;return -1;}
  c->value=c->id==V4L2_CID_FOCUS_AUTO?automatic:position;return 0;
 }
 errno=ENOTSUP;return -1;
}
#define ioctl fake_ioctl
#define main capture_program_main
#include "capture.c"
#undef main
#undef ioctl
static void reset_driver(void){
 automatic=1;position=68;maximum=1023;step=1;flags=0;
 ignore_auto=ignore_position=0;query_fail=set_fail=get_fail=0;write_count=0;errno=0;
}
static int checks,errors;
#define CHECK(expr) do{checks++;if(!(expr)){errors++;printf("FAIL: line %d: %s\n",__LINE__,#expr);}}while(0)
int main(void){
 reset_driver();CHECK(lock_focus(1,260)==0&&automatic==0&&position==260);
 CHECK(write_count==2&&writes[0]==1&&writes[1]==2);
 CHECK(verify_focus(1,260)==0);
 position=261;CHECK(verify_focus(1,260)<0&&errno==EPROTO);
 position=260;automatic=1;CHECK(verify_focus(1,260)<0&&errno==EPROTO);
 reset_driver();CHECK(lock_focus(1,0)==0&&position==0);
 reset_driver();CHECK(lock_focus(1,-1)<0&&write_count==0);
 reset_driver();CHECK(lock_focus(1,1024)<0&&write_count==0);
 reset_driver();step=5;CHECK(lock_focus(1,261)<0&&write_count==0);
 reset_driver();query_fail=V4L2_CID_FOCUS_AUTO;CHECK(lock_focus(1,260)<0&&write_count==0);
 reset_driver();query_fail=V4L2_CID_FOCUS_ABSOLUTE;CHECK(lock_focus(1,260)<0&&write_count==0);
 reset_driver();flags=V4L2_CTRL_FLAG_DISABLED;CHECK(lock_focus(1,260)<0&&write_count==0);
 reset_driver();flags=V4L2_CTRL_FLAG_READ_ONLY;CHECK(lock_focus(1,260)<0&&write_count==0);
 reset_driver();set_fail=V4L2_CID_FOCUS_AUTO;CHECK(lock_focus(1,260)<0);
 reset_driver();set_fail=V4L2_CID_FOCUS_ABSOLUTE;CHECK(lock_focus(1,260)<0);
 reset_driver();ignore_auto=1;CHECK(lock_focus(1,260)<0&&errno==EPROTO);
 reset_driver();ignore_position=1;CHECK(lock_focus(1,260)<0&&errno==EPROTO);
 reset_driver();get_fail=V4L2_CID_FOCUS_ABSOLUTE;CHECK(lock_focus(1,260)<0);
 reset_driver();get_fail=V4L2_CID_FOCUS_AUTO;CHECK(lock_focus(1,260)<0);
 int value;CHECK(focus_number("0",&value)==0&&value==0);
 CHECK(focus_number("260",&value)==0&&value==260);
 CHECK(focus_number("-1",&value)<0&&focus_number("260x",&value)<0&&focus_number("",&value)<0);
 printf("%s: %d focus checks, %d errors\n",errors?"FAIL":"PASS",checks,errors);
 return errors?1:0;
}
