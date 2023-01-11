// LWIP Echo example taken from:
// https://coherentmusings.wordpress.com/2012/06/11/tcp-echo-server-with-lwip/
// Slightly modified. So that the names of the functions match the example better.
// Also, there is a possible error on line 26+28. We will cover this in class.

#include <tcp.h>

char tcp_buffer[1024];

static void echo_close (struct tcp_pcb *pcb )
{
    tcp_arg(pcb, NULL);
    tcp_sent(pcb, NULL);
    tcp_recv(pcb, NULL);
    tcp_close(pcb);
}

static err_t echo_recv( void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err )
{
    int i;
    int len;
    char *pc;

    if ( err == ERR_OK && p != NULL )
    {
        tcp_recved( pcb, p->tot_len );  //possible error
        pc = (char *)p->payload;
        len =p->tot_len;				//possible error

        for( i=0; i<len; i++ )
        {
            tcp_buffer[i] = pc[i];
        }

        if( tcp_buffer[0] == 'X' )
            echo_close( pcb );

        pbuf_free( p );

        if( len > tcp_sndbuf( pcb ) )
            len= tcp_sndbuf( pcb );

        tcp_write( pcb, tcp_buffer, len, 0 );
        tcp_sent( pcb, NULL );
    }
    else
    {
        pbuf_free( p );
    }

    if( err == ERR_OK && p == NULL )
    {
        echo_close( pcb );
    }

    return ERR_OK;
}

static err_t echo_accept(void *arg, struct tcp_pcb *pcb, err_t err )
{
    LWIP_UNUSED_ARG( arg );
    LWIP_UNUSED_ARG( err );
    tcp_setprio( pcb, TCP_PRIO_MIN );
    tcp_recv( pcb, echo_recv );
    tcp_err( pcb, NULL );
    tcp_poll( pcb, NULL, 4 );
    tcp_write( pcb,
    		"    Welcome to the echo server demo    \r\n"
    		" (I send everything back! X to close)  \r\n"
    		"=======================================\r\n", 118, 0 );
    tcp_sent( pcb, NULL );
    return ERR_OK;
}

void echo_init( void )
{
    struct tcp_pcb *tcp_pcb;
    tcp_pcb = tcp_new();
    tcp_bind(tcp_pcb, IP_ADDR_ANY, 23);

    tcp_pcb = tcp_listen( tcp_pcb );
    tcp_accept( tcp_pcb, echo_accept );
}
