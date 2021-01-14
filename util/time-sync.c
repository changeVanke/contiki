#include "mac-util.h"

#include "net/ipv6/simple-udp.h"
#include "lib/random.h"
#include "net/netstack.h"
#include "lib/list.h"
#include DeviceFamily_constructPath(driverlib/aon_rtc.h)

#include "sys/log.h"
#define LOG_MODULE "Hello"
#define LOG_LEVEL LOG_LEVEL_DBG


/*---------------------------------------------------------------------------------------------*/
uip_ipaddr_t get_parent_node() {
    uip_ipaddr_t parent;
    uip_ds6_defrt_t *default_route;
    if(NETSTACK_ROUTING.node_is_reachable() ) {
        default_route = uip_ds6_defrt_lookup(uip_ds6_defrt_choose());
        LOG_INFO("parent ipaddr:");
        LOG_INFO_6ADDR(&default_route->ipaddr);
        LOG_INFO_("\n");
        uip_ipaddr_copy(&parent, &(default_route->ipaddr));
    } else {
        LOG_INFO("can not find the parent node address.\n");
    }
    return parent;
}

/*---------------------------------------------------------------------------------------------*/
uint32_t get_present_time() {

    uint32_t now;
    now = RTIMER_NOW();
    //now = AONRTCCurrent64BitValueGet();
    //clock_time_t ticks = (clock_time_t)(now / (RTC_SUBSEC_FRAC / CLOCK_SECOND));
    return now;
}

