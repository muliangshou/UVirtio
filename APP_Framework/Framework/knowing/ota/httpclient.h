/*
 * @Author: chunyexixiaoyu
 * @Date: 2021-08-26 13:53:03
 * @LastEditTime: 2021-08-30 14:44:05
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \xiuos\APP_Framework\Framework\knowing\ota\httpclient.h
 */
#ifndef _OTA_HTTPCLIENT_H
#define _OTA_HTTPCLIENT_H
#include <transform.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef HTTP_MALLOC
#define HTTP_MALLOC                     malloc
#endif

#ifndef HTTP_CALLOC
#define HTTP_CALLOC                     calloc
#endif

#ifndef HTTP_REALLOC
#define HTTP_REALLOC                    realloc
#endif

#ifndef HTTP_FREE
#define HTTP_FREE                       free
#endif

#ifndef HTTP_STRDUP
#define HTTP_STRDUP                     strdup
#endif

#define HTTPCLIENT_SW_VERSION           "2.2.0"
#define HTTPCLIENT_SW_VERSION_NUM       0x20200

#define HTTPCLIENT_HEADER_BUFSZ         4096
#define HTTPCLIENT_RESPONSE_BUFSZ       4096

#define HTTPCLIENT_DEFAULT_TIMEO        6

enum webClientStatus
{
    HTTPCLIENT_OK,
    HTTPCLIENT_ERROR,
    HTTPCLIENT_TIMEOUT,
    HTTPCLIENT_NOMEM,
    HTTPCLIENT_NOSOCKET,
    HTTPCLIENT_NOBUFFER,
    HTTPCLIENT_CONNECT_FAILED,
    HTTPCLIENT_DISCONNECT,
    HTTPCLIENT_FILE_ERROR,
};

enum httpClientMethod
{
    WEBCLIENT_USER_METHOD,
    WEBCLIENT_GET,
    WEBCLIENT_POST,
};

struct  httpClientHeader
{
    char *buffer;
    size_t length;                      /* content header buffer size */

    size_t size;                        /* maximum support header size */
};

struct httpClientSession
{
    struct httpClientHeader *header;    /* webclient response header information */
    int socket;
    int resp_status;

    char *host;                         /* server host */
    char *req_url;                      /* HTTP request address*/

    int chunk_sz;
    int chunk_offset;

    int content_length;
    size_t content_remainder;           /* remainder of content length */

    int is_tls;                   /* HTTPS connect */
};

/* create webclient session and set header response size */
struct httpClientSession *httpClientSessionCreate(size_t header_sz);

/* send HTTP GET request */
int httpClientGet(struct httpClientSession *session, const char *URI);
int httpClientGetPosition(struct httpClientSession *session, const char *URI, int position);

/* send HTTP POST request */
int httpClientPost(struct httpClientSession *session, const char *URI, const void *post_data, size_t data_len);

/* close and release wenclient session */
int httpClientClose(struct httpClientSession *session);

int httpClientSetTimeout(struct httpClientSession *session, int millisecond);

/* send or receive data from server */
int httpClientRead(struct httpClientSession *session, void *buffer, size_t size);
int httpClientWrite(struct httpClientSession *session, const void *buffer, size_t size);

/* webclient GET/POST header buffer operate by the header fields */
int httpClientHeaderFieldsAdd(struct httpClientSession *session, const char *fmt, ...);
const char *httpClientHeaderFieldsGet(struct httpClientSession *session, const char *fields);

/* send HTTP POST/GET request, and get response data */
int httpClientResponse(struct httpClientSession *session, void **response, size_t *resp_len);
int httpClientRequest(const char *URI, const char *header, const void *post_data, size_t data_len, void **response, size_t *resp_len);
int httpClientRequestHeaderAdd(char **request_header, const char *fmt, ...);
int httpClientRespStatusGet(struct httpClientSession *session);
int httpClientContentLengthGet(struct httpClientSession *session);

/*
file system must be supported
*/
/* file related operations */
int httpClientGetFile(const char *URI, const char *filename);
int httpClientPostFile(const char *URI, const char *filename, const char *form_data);

extern long int strtol(const char *nptr, char **endptr, int base);
#ifdef  __cplusplus
    }
#endif
#endif