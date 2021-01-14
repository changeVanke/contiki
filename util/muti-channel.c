#include "mac-util.h"

#include <stdlib.h>
#include <stdio.h>

#include "net/netstack.h"
#include "net/packetbuf.h"
/*---------------------------------------------------------------------------------------------*/
void get_node_key_channel_by_rand(void)
{
    int rand;
    int radio_addr;
    /* get the local device's 16bitt address. */
    NETSTACK_RADIO.get_value(RADIO_PARAM_16BIT_ADDR, &radio_addr);

    /* call the function of generating pseudo random number, and use it select the device's working channel. */
    printf("RADIO 16bit ADDRESS IS hexadecimal is %x and binary is %d\n", radio_addr, radio_addr);

    /* get the channel selected num by radio 16bit address, and set the value to RADIO_PARAM_CHANNEL. */
    if (radio_addr)
    {
       rand = get_rand_num(radio_addr);
       if(rand != 0) {
           NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, rand);
       }
    }
    else {
        /* default channel。 */
        NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, 0);
    }
}

/*---------------------------------------------------------------------------------------------*/
int get_rand_num(uint16_t source) {
    /* num >= 2 && num <= 9. */
    int num;
    /* to get the rand() source from 'source'. */
    srand(source);
    /* rand() % 8 get the number which is from 0 to 7, and plus 2 to get the num from 2 to 9. */
    num = rand() % 8 + 2;
    return num;
}

/*---------------------------------------------------------------------------------------------*/
void catch_frame(void) {

    frame802154_t frame;
    int len;
    linkaddr_t source_addr;
    linkaddr_t dest_addr;
    int i = 0;

    len = packetbuf_datalen();
    if(frame802154_parse(packetbuf_dataptr(), len, &frame)) {
        printf("*****parse the frame*****\n");
        frame802154_extract_linkaddr(&frame, &source_addr, &dest_addr);
        while(i < 8) {
            printf("%d\n", source_addr.u8[i]);
            i++;
        }
        printf("******************");
        i = 0;
        while(i < 8) {
            printf("%d\n", dest_addr.u8[i]);
            i++;
        }
        //printf("\n");
    }
}
