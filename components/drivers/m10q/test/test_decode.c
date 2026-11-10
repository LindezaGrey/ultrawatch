#include "m10q_decode.h"
#include <assert.h>
#include <math.h>
#include <string.h>
static void put(uint8_t *p, uint32_t n) { for (unsigned i=0;i<4;i++) p[i]=(uint8_t)(n>>(i*8)); }
int main(void) {
    m10q_fix_t f={0};
    uint8_t sat[8+12*30]={0}; sat[4]=1; sat[5]=30;
    for(unsigned i=0;i<30;i++) {
        uint8_t *s=sat+8+12*i; s[0]=i%7; s[1]=i+1; s[2]=i+15; s[3]=45;
        s[8]=4 | (i%2?8:0);
    }
    assert(m10q_decode_sat(&f,sat,sizeof(sat),100));
    assert(f.sats_total==30 && f.sat_in_view==24 && f.sats_above_30==15 && f.sats_above_25==20);
    assert(f.sats[0].prn==30 && f.sats[0].used && !f.sats[1].used && f.sats[0].gnss_id==1);
    assert(!m10q_decode_sat(&f,sat,sizeof(sat),200) && f.sats_updated_ms==100);
    assert(!m10q_decode_sat(&f,sat,sizeof(sat)-1,200));
    sat[4]=2; assert(!m10q_decode_sat(&f,sat,sizeof(sat),200)); sat[4]=1;
    uint8_t p[92]={0}; p[20]=2; p[21]=1; p[23]=12;
    put(p+36,INT32_MIN); put(p+60,1000); put(p+64,9000000);
    p[4]=0xea; p[5]=7; p[6]=9; p[7]=9; p[11]=3;
    assert(m10q_decode_pvt(&f,p,sizeof(p),100));
    assert(f.valid && !f.fix_3d && isnan(f.alt_m) && f.lat==0 && f.lon==0);
    assert(f.heading_valid && f.course_deg==90 && f.time_valid);
    assert(!m10q_decode_pvt(&f,p,sizeof(p),200) && f.updated_ms==100);
    put(p,1000);p[20]=3;put(p+36,123456);put(p+60,0);
    assert(m10q_decode_pvt(&f,p,sizeof(p),1000));
    assert(f.valid && f.fix_3d && fabs(f.alt_m-123.456)<0.0001 && !f.heading_valid);
    uint8_t d[18]={0}; d[12]=125;
    assert(m10q_decode_dop(&f,d,sizeof(d),1000) && f.hdop==13);
    m10q_expire_snapshot(&f,3099);assert(f.valid && f.sats_valid);
    m10q_expire_snapshot(&f,3100);assert(f.valid && !f.sats_valid && !f.sat_in_view);
    m10q_expire_snapshot(&f,4000);assert(!f.valid && isnan(f.alt_m) && f.hdop==UINT16_MAX);
    put(p,2000);p[21]=0;assert(m10q_decode_pvt(&f,p,sizeof(p),5000) && !f.valid);
    put(p,3000);p[21]=1;put(p+28,910000000);
    assert(m10q_decode_pvt(&f,p,sizeof(p),6000) && !f.valid);
    memset(&f,0,sizeof(f));put(p+28,0);
    assert(m10q_decode_pvt(&f,p,sizeof(p),UINT32_MAX-1000));
    m10q_expire_snapshot(&f,1998);assert(f.valid);
    m10q_expire_snapshot(&f,1999);assert(!f.valid);
    return 0;
}
