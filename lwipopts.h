/* lwipopts.h — Pico W, cyw43_arch_lwip_threadsafe_background (NO_SYS = 1) */
#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

/* ================= System ================= */
#define NO_SYS                          1   /* <- REQUIRED for threadsafe_background */
#define LWIP_TIMERS                     1
#define LWIP_TIMERS_CUSTOM              0
#define SYS_LIGHTWEIGHT_PROT            1
#define LWIP_PROVIDE_ERRNO              1

/* ============= Memory / Buffers =========== */
#define MEM_ALIGNMENT                   4
#define MEM_SIZE                        (32 * 1024)
#define MEMP_NUM_PBUF                   16
#define MEMP_NUM_TCP_PCB                5
#define MEMP_NUM_TCP_SEG                16
#define MEMP_NUM_SYS_TIMEOUT            10

#define PBUF_POOL_SIZE                  16
#define PBUF_POOL_BUFSIZE               1536

#define MEMP_MEM_MALLOC                 1
#define LWIP_ALLOW_MEM_FREE_FROM_OTHER_CONTEXT 1

/* ============== Protocols ================= */
#define LWIP_IPV4                       1
#define LWIP_IPV6                       0
#define LWIP_ETHERNET                   1
#define LWIP_ARP                        1
#define LWIP_ICMP                       1
#define LWIP_RAW                        1
#define LWIP_UDP                        1
#define LWIP_TCP                        1
#define LWIP_DHCP                       1
#define LWIP_DNS                        1
#define LWIP_SNTP                       1

/* ============== TCP tuning ================ */
#define TCP_MSS                         1460
#define TCP_SND_BUF                     (8 * TCP_MSS)
#define TCP_WND                         (8 * TCP_MSS)
#define TCP_SND_QUEUELEN                (2 * TCP_SND_BUF / TCP_MSS)
#define TCP_QUEUE_OOSEQ                 0
#define TCP_TTL                         255
#define LWIP_TCP_KEEPALIVE              1
#define LWIP_TCP_TIMESTAMPS             0

/* ============== API Layers =================
   NO_SYS=1 => RAW API only. Disable NETCONN/SOCKETS and TCPIP thread stuff. */
#define LWIP_NETCONN                    0
#define LWIP_SOCKET                     0
#define LWIP_NETIF_API                  0
#define LWIP_NETIF_STATUS_CALLBACK      1
#define LWIP_NETIF_LINK_CALLBACK        1

/* DO NOT define TCPIP_THREAD_* when NO_SYS=1 */
 /* (intentionally omitted) */

/* ============== DHCP/DNS ================== */
#define LWIP_DHCP_CHECK_LINK_UP         1
#define LWIP_DHCP_DOES_ARP_CHECK        1
#define LWIP_DHCP_MAX_TRIES             5
#define LWIP_DNS_SECURE                 7
#define DNS_MAX_NAME_LENGTH             64
#define DNS_MAX_SERVERS                 2

/* ============== Checksums ================= */
#define CHECKSUM_BY_HARDWARE            0
#define LWIP_CHECKSUM_ON_COPY           1
#define CHECKSUM_GEN_IP                 1
#define CHECKSUM_GEN_UDP                1
#define CHECKSUM_GEN_TCP                1
#define CHECKSUM_GEN_ICMP               1
#define CHECKSUM_CHECK_IP               1
#define CHECKSUM_CHECK_UDP              1
#define CHECKSUM_CHECK_TCP              1
#define CHECKSUM_CHECK_ICMP             1

/* ============== Debug/Stats =============== */
#define LWIP_DEBUG                      0
#define LWIP_STATS                      0
#define LWIP_STATS_DISPLAY              0

/* ========== Pico W (CYW43) glue =========== */
#define CYW43_LWIP                      1
/* Don’t force CYW43_USE_FREERTOS/THREAD_SAFE here—arch selection is done
   in CMake by linking pico_cyw43_arch_lwip_threadsafe_background. */

#endif /* __LWIPOPTS_H__ */
