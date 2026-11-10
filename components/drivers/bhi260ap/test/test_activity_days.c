#include "activity_days.h"
#include "daily_archive.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
int main(void)
{
    activity_days_t d = {.version=ACTIVITY_DAYS_VERSION};
    int64_t since=0;
    assert(activity_days_advance(&d,20260909,&since,1000000));
    activity_day_steps(&d.current,100);
    activity_day_credit(&d.current,1,&since,61000000);
    assert(d.current.steps==100 && d.current.ms[1]==60000);
    /* Same-day boot restores one consistent checkpoint; off time is excluded. */
    activity_days_t reboot=d;
    since=0;
    assert(activity_days_advance(&reboot,20260909,&since,0));
    activity_day_steps(&reboot.current,105);
    activity_day_credit(&reboot.current,0,&since,2000000);
    assert(reboot.current.steps==105 && reboot.current.ms[1]==60000 && reboot.current.ms[0]==2000);
    /* Midnight archives the live segment and restarts its clock. */
    assert(activity_days_advance(&d,20260910,&since,100000000));
    assert(d.pending_count==1 && d.pending[0].steps==100 && !d.current.steps);
    activity_day_credit(&d.current,1,&since,102000000);
    assert(d.current.ms[1]==2000); /* yesterday's elapsed time must not return */
    activity_day_steps(&d.current,107);assert(d.current.steps==7);
    /* Several powered-off days create one summary, no invented zero/activity days. */
    assert(activity_days_advance(&d,20260914,&since,0));
    assert(d.pending_count==2 && d.pending[1].ymd==20260910 && d.current.ms[1]==0);
    activity_days_t unchanged=d;
    assert(!activity_days_advance(&d,20260913,&since,0));
    assert(memcmp(&d,&unchanged,sizeof(d))==0);
    /* A legacy counter rollback rebases instead of suppressing future steps. */
    activity_day_steps(&d.current,50);activity_day_steps(&d.current,53);
    assert(d.current.steps==3);
    while(d.pending_count<ACTIVITY_PENDING_DAYS) {
        assert(activity_days_advance(&d,d.current.ymd+1,&since,0));
    }
    unchanged=d;
    assert(!activity_days_advance(&d,d.current.ymd+1,&since,0));
    assert(memcmp(&d,&unchanged,sizeof(d))==0);
    /* Failed SD writes leave the durable outbox untouched; retries overwrite one file. */
    assert(!daily_archive_write("/missing/activity-test-dir",&d.pending[0]));
    assert(d.pending_count==ACTIVITY_PENDING_DAYS);
    char directory[]="/tmp/uwatch-activity-XXXXXX";
    assert(mkdtemp(directory));
    assert(daily_archive_write(directory,&d.pending[0]));
    assert(daily_archive_write(directory,&d.pending[0]));
    char path[160], contents[1024]={0};
    snprintf(path,sizeof(path),"%s/%08u.csv",directory,d.pending[0].ymd);
    FILE *f=fopen(path,"r");assert(f);
    size_t size=fread(contents,1,sizeof(contents)-1,f);assert(size>0);fclose(f);
    assert(strstr(contents,"2026-09-09,,100,100,0,60,0,0,0,0,0\n"));
    unsigned lines=0;for(size_t i=0;i<size;i++) if(contents[i]=='\n') lines++;
    assert(lines==2);
    activity_days_ack(&d);assert(d.pending_count==ACTIVITY_PENDING_DAYS-1);
    assert(d.pending[0].ymd==20260910);
    unlink(path);rmdir(directory);
    activity_day_t fractional = {0};
    since=0;
    activity_day_credit(&fractional,0,&since,499);
    activity_day_credit(&fractional,0,&since,1001);
    assert(fractional.ms[0]==1 && since==1000);
    /* Host sleep credits awake time before sleep, not the sleep interval. */
    activity_day_t sleep_day = {0};
    since=0;
    activity_day_pause(&sleep_day,2,&since,10000000);
    activity_day_resume(&since,110000000);
    activity_day_credit(&sleep_day,2,&since,115000000);
    assert(sleep_day.ms[2]==15000);
    return 0;
}
