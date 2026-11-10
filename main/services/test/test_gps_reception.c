#include "gps_reception.h"
#include <assert.h>
int main(void) {
    gps_reception_t s;
    m10q_fix_t f = {0};
    gps_reception_start(&s, 0);
    assert(gps_reception_update(&s, 299999, &f) == GPS_RECEPTION_OK);
    assert(gps_reception_update(&s, 300000, &f) == GPS_NO_TELEMETRY);
    gps_reception_start(&s, 0);
    for (unsigned t=1000;t<=600000;t+=1000) {
        f.sats_valid=true; f.sats_updated_ms=t; f.sats_above_25=f.sats_above_30=4;
        assert(gps_reception_update(&s,t,&f)==(t==600000?GPS_NO_FIX:GPS_RECEPTION_OK));
    }
    gps_reception_start(&s, 0);
    f=(m10q_fix_t){.sats_valid=true};
    for(unsigned t=1000;t<=300000;t+=1000) {
        f.sats_updated_ms=t;
        assert(gps_reception_update(&s,t,&f)==(t==300000?GPS_POOR_SIGNAL:GPS_RECEPTION_OK));
    }
    gps_reception_start(&s, UINT32_MAX-300000u);
    f=(m10q_fix_t){0};
    assert(gps_reception_update(&s,UINT32_MAX-1u,&f)==GPS_RECEPTION_OK);
    assert(gps_reception_update(&s,0,&f)==GPS_NO_TELEMETRY);
    gps_reception_start(&s,0);
    f=(m10q_fix_t){.valid=true,.pvt_received=true,.updated_ms=299000};
    assert(gps_reception_update(&s,300000,&f)==GPS_RECEPTION_OK);
    assert(gps_reception_update(&s,599000,&f)==GPS_NO_TELEMETRY);
    gps_reception_start(&s,0);
    f=(m10q_fix_t){.sats_valid=true,.sats_above_25=4,.sats_above_30=4};
    for(unsigned t=1000;t<=5000;t+=1000) {
        f.sats_updated_ms=t;
        assert(gps_reception_update(&s,t,&f)==GPS_RECEPTION_OK);
    }
    assert(s.good);
    /* Cached reports must not renew the timer or count as new samples. */
    assert(gps_reception_update(&s,305000,&f)==GPS_NO_TELEMETRY);
    gps_reception_start(&s,0);
    for(unsigned t=1000;t<=10000;t+=1000) {
        f.sats_updated_ms=t; f.sats_above_30=t<=5000?4:0;
        assert(gps_reception_update(&s,t,&f)==GPS_RECEPTION_OK);
    }
    assert(s.good); /* 25-29 dBHz retains the established good state. */
    for(unsigned t=11000;t<=15000;t+=1000) {
        f.sats_updated_ms=t; f.sats_above_25=0;
        assert(gps_reception_update(&s,t,&f)==GPS_RECEPTION_OK);
    }
    assert(!s.good);
    f.sats_valid=true; f.sats_updated_ms=315000;
    assert(gps_reception_update(&s,315000,&f)==GPS_POOR_SIGNAL);
    gps_reception_start(&s,315000);
    assert(gps_reception_update(&s,315000,&f)==GPS_RECEPTION_OK);
    return 0;
}
