/*
 * @Author: chunyexixiaoyu
 * @Date: 2021-08-20 13:52:54
 * @LastEditTime: 2021-08-30 15:12:42
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * 
 * 
 * @FilePath: \xiuos\APP_Framework\Framework\knowing\ota\httpclient.c
 */
#include <httpclient.h>
static int httpClientStrncaseCmp(const char *a, const char *b, size_t n)
{
    uint8_t c1, c2;
    if (n <= 0)
        return 0;
    do {
        c1 = tolower(*a++);
        c2 = tolower(*b++);
    } while (--n && c1 && c1 == c2);
    return c1 - c2;
}

static const char *webclient_strstri(const char* str, const char* subStr)
{
    int len = strlen(subStr);

    if(len == 0)
    {
        return RT_NULL;
    }

    while(*str)
    {
        if(httpClientStrncaseCmp(str, subStr, len) == 0)
        {
            return str;
        }
        ++str;
    }
    return RT_NULL;
}

static int httpClientSend(struct httpClientSession* session, const void *buffer, size_t len, int flag)
{
    return send(session->socket, buffer, len, flag);
}

static int httpClientRecv(struct httpClientSession* session, void *buffer, size_t len, int flag)
{

    return recv(session->socket, buffer, len, flag);
}

static int httpClientReadLine(struct httpClientSession *session, char *buffer, int size)
{
    int rc, count = 0;
    char ch = 0, last_ch = 0;
    assert(session);
    assert(buffer);

    /* Keep reading until we fill the buffer. */
    while (count < size)
    {
        rc = httpClientRecv(session, (unsigned char *) &ch, 1, 0);
        if (rc <= 0)
            return rc;

        if (ch == '\n' && last_ch == '\r')
            break;

        buffer[count++] = ch;

        last_ch = ch;
    }

    if (count > size)
    {
        printf("read line failed. The line data length is out of buffer size(%d)!", count);
        return -HTTPCLIENT_ERROR;
    }

    return count;
}

/**
 * resolve server address
 *
 * @param session http session
 * @param res the server address information
 * @param url the input server URI address
 * @param request the pointer to point the request url, for example, /index.html
 *
 * @return 0 on resolve server address OK, others failed
*/
static int httpClientResolveaddr(struct httpClientSession *session, struct addrinfo **res,
                                     const char *url, const char **request)
{
    int rc = HTTPCLIENT_OK;
    char *ptr;
    char port_str[6] = "80"; /* default port of 80(http) */
    const char *port_ptr;
    const char *path_ptr;

    const char *host_addr = 0;
    int url_len, host_addr_len = 0;

    assert(res);
    assert(request);

    url_len = strlen(url);

    /* strip protocol(http or https) */
    if (strncmp(url, "http://", 7) == 0)
    {
        host_addr = url + 7;
    }
    else if (strncmp(url, "https://", 8) == 0)
    {
        strncpy(port_str, "443", 4);
        host_addr = url + 8;
    }
    else
    {
        rc = -HTTPCLIENT_ERROR;
        goto __exit;
    }

    /* ipv6 address */
    if (host_addr[0] == '[')
    {
        host_addr += 1;
        ptr = strstr(host_addr, "]");
        if (!ptr)
        {
            rc = -HTTPCLIENT_ERROR;
            goto __exit;
        }
        host_addr_len = ptr - host_addr;
    }

    path_ptr = strstr(host_addr, "/");
    *request = path_ptr ? path_ptr : "/";

    /* resolve port */
    port_ptr = strstr(host_addr + host_addr_len, ":");
    if (port_ptr && path_ptr && (port_ptr < path_ptr))
    {
        int port_len = path_ptr - port_ptr - 1;

        strncpy(port_str, port_ptr + 1, port_len);
        port_str[port_len] = '\0';
    }

    if (port_ptr && (!path_ptr))
    {
        strcpy(port_str, port_ptr + 1);
    }

    /* ipv4 or domain. */
    if (!host_addr_len)
    {
        if (port_ptr)
        {
            host_addr_len = port_ptr - host_addr;
        }
        else if (path_ptr)
        {
            host_addr_len = path_ptr - host_addr;
        }
        else
        {
            host_addr_len = strlen(host_addr);
        }
    }

    if ((host_addr_len < 1) || (host_addr_len > url_len))
    {
        rc = -HTTPCLIENT_ERROR;
        goto __exit;
    }

    /* get host address ok. */
    {
        char *host_addr_new = HTTP_MALLOC(host_addr_len + 1);

        if (!host_addr_new)
        {
            rc = -HTTPCLIENT_ERROR;
            goto __exit;
        }

        memcpy(host_addr_new, host_addr, host_addr_len);
        host_addr_new[host_addr_len] = '\0';
        session->host = host_addr_new;
    }

    printf("host address: %s , port: %s \n\r", session->host, port_str);

    /* resolve the host name. */
    {
        struct addrinfo hint;
        int ret;

        rt_memset(&hint, 0, sizeof(hint));
        ret = getaddrinfo(session->host, port_str, &hint, res);
        if (ret != 0)
        {
            printf("getaddrinfo err: %d '%s'.", ret, session->host);
            rc = -HTTPCLIENT_ERROR;
            goto __exit;
        }
    }

__exit:
    if (rc != HTTPCLIENT_OK)
    {
        if (session->host)
        {
            HTTP_FREE(session->host);
            session->host = RT_NULL;
        }

        if (*res)
        {
            freeaddrinfo(*res);
            *res = RT_NULL;
        }
    }

    return rc;
}


/**
 * connect to http server.
 *
 * @param session http session
 * @param URI the input server URI address
 *
 * @return <0: connect failed or other error
 *         =0: connect success
 */
static int httpClientConnect(struct httpClientSession *session, const char *URI)
{
    int rc = HTTPCLIENT_OK;
    int socket_handle;
    struct timeval timeout;
    struct addrinfo *res = RT_NULL;
    const char *req_url;

    assert(session);
    assert(URI);

    timeout.tv_sec = HTTPCLIENT_DEFAULT_TIMEO;
    timeout.tv_usec = 0;

    if (strncmp(URI, "https://", 8) == 0)
    {

        printf("not support https connect, please enable webclient https configure!");
        rc = -HTTPCLIENT_ERROR;
        goto __exit;
    }

    /* Check valid IP address and URL */
    rc = httpClientResolveaddr(session, &res, URI, &req_url);
    if (rc != HTTPCLIENT_OK)
    {
        printf("connect failed, resolve address error(%d).", rc);
        goto __exit;
    }

    /* Not use 'getaddrinfo()' for https connection */
    if (session->is_tls == RT_FALSE && res == RT_NULL)
    {
        rc = -HTTPCLIENT_ERROR;
        goto __exit;
    }

    /* copy host address */
    if (req_url)
    {
        session->req_url = HTTP_STRDUP(req_url);
    }
    else
    {
        printf("connect failed, resolve request address error.");
        rc = -HTTPCLIENT_ERROR;
        goto __exit;
    }


        socket_handle = socket(res->ai_family, SOCK_STREAM, IPPROTO_TCP);

        if (socket_handle < 0)
        {
            printf("connect failed, create socket(%d) error.", socket_handle);
            rc = -HTTPCLIENT_NOSOCKET;
            goto __exit;
        }

        /* set receive and send timeout option */
        setsockopt(socket_handle, SOL_SOCKET, SO_RCVTIMEO, (void *) &timeout,
                   sizeof(timeout));
        setsockopt(socket_handle, SOL_SOCKET, SO_SNDTIMEO, (void *) &timeout,
                   sizeof(timeout));

        if (connect(socket_handle, res->ai_addr, res->ai_addrlen) != 0)
        {
            /* connect failed, close socket */
            printf("connect failed, connect socket(%d) error.", socket_handle);
            closesocket(socket_handle);
            rc = -HTTPCLIENT_CONNECT_FAILED;
            goto __exit;
        }

        session->socket = socket_handle;
    

__exit:
    if (res)
    {
        freeaddrinfo(res);
    }

    return rc;
}

/**
 * add fields data to request header data.
 *
 * @param session http session
 * @param fmt fields format
 *
 * @return >0: data length of successfully added
 *         <0: not enough header buffer size
 */
int httpClientHeaderFieldsAdd(struct httpClientSession *session, const char *fmt, ...)
{
    rt_int32_t length;
    va_list args;

    assert(session);
    assert(session->header->buffer);

    va_start(args, fmt);
    length = rt_vsnprintf(session->header->buffer + session->header->length,
            session->header->size - session->header->length, fmt, args);
    if (length < 0)
    {
        printf("add fields header data failed, return length(%d) error.", length);
        return -HTTPCLIENT_ERROR;
    }
    va_end(args);

    session->header->length += length;

    /* check header size */
    if (session->header->length >= session->header->size)
    {
        printf("not enough header buffer size(%d)!", session->header->size);
        return -HTTPCLIENT_ERROR;
    }

    return length;
}

/**
 * get fields information from request/response header data.
 *
 * @param session http session
 * @param fields fields keyword
 *
 * @return = NULL: get fields data failed
 *        != NULL: success get fields data
 */
const char *httpClientHeaderFieldsGet(struct httpClientSession *session, const char *fields)
{
    char *resp_buf = RT_NULL;
    size_t resp_buf_len = 0;

    assert(session);
    assert(session->header->buffer);

    resp_buf = session->header->buffer;
    while (resp_buf_len < session->header->length)
    {
        if (webclient_strstri(resp_buf, fields) == resp_buf)
        {
            char *mime_ptr = RT_NULL;

            /* jump space */
            mime_ptr = rt_strstr(resp_buf, ":");
            if (mime_ptr != NULL)
            {
                mime_ptr += 1;

                while (*mime_ptr && (*mime_ptr == ' ' || *mime_ptr == '\t'))
                    mime_ptr++;

                return mime_ptr;
            }
        }

        if (*resp_buf == '\0')
            break;

        resp_buf += rt_strlen(resp_buf) + 1;
        resp_buf_len += rt_strlen(resp_buf) + 1;
    }

    return RT_NULL;
}

/**
 * get http response status code.
 *
 * @param session http session
 *
 * @return response status code
 */
int httpClientRespStatusGet(struct httpClientSession *session)
{
    assert(session);

    return session->resp_status;
}

/**
 * get http response data content length.
 *
 * @param session http session
 *
 * @return response content length
 */
int httpClientContentLengthGet(struct httpClientSession *session)
{
    assert(session);

    return session->content_length;
}

static int httpClientSendHeader(struct httpClientSession *session, int method)
{
    int rc = HTTPCLIENT_OK;
    char *header = RT_NULL;

    assert(session);

    header = session->header->buffer;

    if (session->header->length == 0)
    {
        /* use default header data */
        if (httpClientHeaderFieldsAdd(session, "GET %s HTTP/1.1\r\n", session->req_url) < 0)
            return -HTTPCLIENT_NOMEM;
        if (httpClientHeaderFieldsAdd(session, "Host: %s\r\n", session->host) < 0)
            return -HTTPCLIENT_NOMEM;
        if (httpClientHeaderFieldsAdd(session, "User-Agent: XIUOS HTTP Agent\r\n\r\n") < 0)
            return -HTTPCLIENT_NOMEM;

        httpClientWrite(session, (unsigned char *) session->header->buffer, session->header->length);
    }
    else
    {
        if (method != WEBCLIENT_USER_METHOD)
        {
            /* check and add fields header data */
            if (memcmp(header, "HTTP/1.", rt_strlen("HTTP/1.")))
            {
                char *header_buffer = RT_NULL;
                int length = 0;

                header_buffer = HTTP_STRDUP(session->header->buffer);
                if (header_buffer == RT_NULL)
                {
                    printf("no memory for header buffer!");
                    rc = -HTTPCLIENT_NOMEM;
                    goto __exit;
                }

                /* splice http request header data */
                if (method == WEBCLIENT_GET)
                    length = rt_snprintf(session->header->buffer, session->header->size, "GET %s HTTP/1.1\r\n%s",
                            session->req_url ? session->req_url : "/", header_buffer);
                else if (method == WEBCLIENT_POST)
                    length = rt_snprintf(session->header->buffer, session->header->size, "POST %s HTTP/1.1\r\n%s",
                            session->req_url ? session->req_url : "/", header_buffer);
                session->header->length = length;

                HTTP_FREE(header_buffer);
            }

            if (strstr(header, "Host:") == RT_NULL)
            {
                if (httpClientHeaderFieldsAdd(session, "Host: %s\r\n", session->host) < 0)
                    return -HTTPCLIENT_NOMEM;
            }

            if (strstr(header, "User-Agent:") == RT_NULL)
            {
                if (httpClientHeaderFieldsAdd(session, "User-Agent: RT-Thread HTTP Agent\r\n") < 0)
                    return -HTTPCLIENT_NOMEM;
            }

            if (strstr(header, "Accept:") == RT_NULL)
            {
                if (httpClientHeaderFieldsAdd(session, "Accept: */*\r\n") < 0)
                    return -HTTPCLIENT_NOMEM;
            }

            /* header data end */
            rt_snprintf(session->header->buffer + session->header->length, session->header->size - session->header->length, "\r\n");
            session->header->length += 2;

            /* check header size */
            if (session->header->length > session->header->size)
            {
                printf("send header failed, not enough header buffer size(%d)!", session->header->size);
                rc = -HTTPCLIENT_NOBUFFER;
                goto __exit;
            }

            httpClientWrite(session, (unsigned char *) session->header->buffer, session->header->length);
        }
        else
        {
            httpClientWrite(session, (unsigned char *) session->header->buffer, session->header->length);
        }
    }

    /* get and echo request header data */
    {
        char *header_str, *header_ptr;
        int header_line_len;
        printf("request header:\n\r");

        for(header_str = session->header->buffer; (header_ptr = rt_strstr(header_str, "\r\n")) != RT_NULL; )
        {
            header_line_len = header_ptr - header_str;

            if (header_line_len > 0)
            {
                printf("%.*s", header_line_len, header_str);
            }
            header_str = header_ptr + rt_strlen("\r\n");
        }

    }

__exit:
    return rc;
}

/**
 * resolve server response data.
 *
 * @param session http session
 *
 * @return <0: resolve response data failed
 *         =0: success
 */
int httpClientHandleResponse(struct httpClientSession *session)
{
    int rc = HTTPCLIENT_OK;
    char *mime_buffer = RT_NULL;
    char *mime_ptr = RT_NULL;
    const char *transfer_encoding;
    int i;

    assert(session);

    /* clean header buffer and size */
    rt_memset(session->header->buffer, 0x00, session->header->size);
    session->header->length = 0;

    printf("response header:\n\r");
    /* We now need to read the header information */
    while (1)
    {
        mime_buffer = session->header->buffer + session->header->length;

        /* read a line from the header information. */
        rc = httpClientReadLine(session, mime_buffer, session->header->size - session->header->length);
        if (rc < 0)
            break;

        /* End of headers is a blank line.  exit. */
        if (rc == 0)
            break;
        if ((rc == 1) && (mime_buffer[0] == '\r'))
        {
            mime_buffer[0] = '\0';
            break;
        }

        /* set terminal charater */
        mime_buffer[rc - 1] = '\0';

        /* echo response header data */
        printf("%s", mime_buffer);

        session->header->length += rc;

        if (session->header->length >= session->header->size)
        {
            printf("not enough header buffer size(%d)!", session->header->size);
            return -HTTPCLIENT_NOMEM;
        }
    }

    /* get HTTP status code */
    mime_ptr = HTTP_STRDUP(session->header->buffer);
    if (mime_ptr == RT_NULL)
    {
        printf("no memory for get http status code buffer!");
        return -HTTPCLIENT_NOMEM;
    }

    if (rt_strstr(mime_ptr, "HTTP/1."))
    {
        char *ptr = mime_ptr;

        ptr += rt_strlen("HTTP/1.x");

        while (*ptr && (*ptr == ' ' || *ptr == '\t'))
            ptr++;

        /* Terminate string after status code */
        for (i = 0; ((ptr[i] != ' ') && (ptr[i] != '\t')); i++);
        ptr[i] = '\0';

        session->resp_status = (int) strtol(ptr, RT_NULL, 10);
    }

    /* get content length */
    if (httpClientHeaderFieldsGet(session, "Content-Length") != RT_NULL)
    {
        session->content_length = atoi(httpClientHeaderFieldsGet(session, "Content-Length"));
    }
    session->content_remainder = session->content_length ? (size_t) session->content_length : 0xFFFFFFFF;

    transfer_encoding = httpClientHeaderFieldsGet(session, "Transfer-Encoding");
    if (transfer_encoding && rt_strcmp(transfer_encoding, "chunked") == 0)
    {
        char line[16];

        /* chunk mode, we should get the first chunk size */
        httpClientReadLine(session, line, session->header->size);
        session->chunk_sz = strtol(line, RT_NULL, 16);
        session->chunk_offset = 0;
    }

    if (mime_ptr)
    {
        HTTP_FREE(mime_ptr);
    }

    if (rc < 0)
    {
        return rc;
    }

    return session->resp_status;
}

/**
 * create http session, set maximum header and response size
 *
 * @param header_sz maximum send header size
 * @param resp_sz maximum response data size
 *
 * @return  http session structure
 */
struct httpClientSession *httpClientSessionCreate(size_t header_sz)
{
    struct httpClientSession *session;

    /* create session */
    session = (struct httpClientSession *) HTTP_CALLOC(1, sizeof(struct httpClientSession));
    if (session == RT_NULL)
    {
        printf("webclient create failed, no memory for webclient session!");
        return RT_NULL;
    }

    /* initialize the socket of session */
    session->socket = -1;
    session->content_length = -1;

    session->header = (struct httpClientHeader *) HTTP_CALLOC(1, sizeof(struct httpClientHeader));
    if (session->header == RT_NULL)
    {
        printf("webclient create failed, no memory for session header!");
        HTTP_FREE(session);
        session = RT_NULL;
        return RT_NULL;
    }

    session->header->size = header_sz;
    session->header->buffer = (char *) HTTP_CALLOC(1, header_sz);
    if (session->header->buffer == RT_NULL)
    {
        printf("webclient create failed, no memory for session header buffer!");
        HTTP_FREE(session->header);
        HTTP_FREE(session);
        session = RT_NULL;
        return RT_NULL;
    }

    return session;
}

static int httpClientClean(struct httpClientSession *session);

/**
 *  send GET request to http server and get response header.
 *
 * @param session http session
 * @param URI input server URI address
 * @param header GET request header
 *             = NULL: use default header data
 *            != NULL: use custom header data
 *
 * @return <0: send GET request failed
 *         >0: response http status code
 */
int httpClientGet(struct httpClientSession *session, const char *URI)
{
    int rc = HTTPCLIENT_OK;
    int resp_status = 0;

    assert(session);
    assert(URI);

    rc = httpClientConnect(session, URI);
    if (rc != HTTPCLIENT_OK)
    {
        /* connect to webclient server failed. */
        return rc;
    }

    rc = httpClientSendHeader(session, WEBCLIENT_GET);
    if (rc != HTTPCLIENT_OK)
    {
        /* send header to webclient server failed. */
       return rc;
    }

    /* handle the response header of webclient server */
    resp_status = httpClientHandleResponse(session);

    printf("get position handle response(%d).", resp_status);

    if (resp_status > 0)
    {
        const char *location = httpClientHeaderFieldsGet(session, "Location");

        /* relocation */
        if ((resp_status == 302 || resp_status == 301) && location)
        {
            char *new_url;

            new_url = HTTP_STRDUP(location);
            if (new_url == RT_NULL)
            {
                return -HTTPCLIENT_NOMEM;
            }

            /* clean webclient session */
            httpClientClean(session);
            /* clean webclient session header */
            session->header->length = 0;
            rt_memset(session->header->buffer, 0, session->header->size);

            rc = httpClientGet(session, new_url);

            HTTP_FREE(new_url);
            return rc;
        }
    }

    return resp_status;
}

/**
 *  http breakpoint resume.
 *
 * @param session http session
 * @param URI input server URI address
 * @param position last downloaded position
 *
 * @return <0: send GET request failed
 *         >0: response http status code
 */
int httpClientGetPosition(struct httpClientSession *session, const char *URI, int position)
{
    int rc = HTTPCLIENT_OK;
    int resp_status = 0;

    assert(session);
    assert(URI);

    rc = httpClientConnect(session, URI);
    if (rc != HTTPCLIENT_OK)
    {
        return rc;
    }

    /* splice header*/
    if (httpClientHeaderFieldsAdd(session, "Range: bytes=%d-\r\n", position) <= 0)
    {
        rc = -HTTPCLIENT_ERROR;
        return rc;
    }

    rc = httpClientSendHeader(session, WEBCLIENT_GET);
    if (rc != HTTPCLIENT_OK)
    {
        return rc;
    }

    /* handle the response header of webclient server */
    resp_status = httpClientHandleResponse(session);

    printf("get position handle response(%d).", resp_status);

    if (resp_status > 0)
    {
        const char *location = httpClientHeaderFieldsGet(session, "Location");

        /* relocation */
        if ((resp_status == 302 || resp_status == 301) && location)
        {
            char *new_url;

            new_url = HTTP_STRDUP(location);
            if (new_url == RT_NULL)
            {
                return -HTTPCLIENT_NOMEM;
            }

            /* clean webclient session */
            httpClientClean(session);
            /* clean webclient session header */
            session->header->length = 0;
            rt_memset(session->header->buffer, 0, session->header->size);

            rc = httpClientGetPosition(session, new_url, position);

            HTTP_FREE(new_url);
            return rc;
        }
    }

    return resp_status;
}

/**
 * send POST request to server and get response header data.
 *
 * @param session http session
 * @param URI input server URI address
 * @param header POST request header, can't be empty
 * @param post_data data send to the server
 *                = NULL: just connect server and send header
 *               != NULL: send header and body data, resolve response data
 * @param data_len the length of send data
 *
 * @return <0: send POST request failed
 *         =0: send POST header success
 *         >0: response http status code
 */
int httpClientPost(struct httpClientSession *session, const char *URI, const void *post_data, size_t data_len)
{
    int rc = HTTPCLIENT_OK;
    int resp_status = 0;

    assert(session);
    assert(URI);

    if ((post_data != RT_NULL) && (data_len == 0))
    {
        printf("input post data length failed");
        return -HTTPCLIENT_ERROR;
    }

    rc = httpClientConnect(session, URI);
    if (rc != HTTPCLIENT_OK)
    {
        /* connect to webclient server failed. */
        return rc;
    }

    rc = httpClientSendHeader(session, WEBCLIENT_POST);
    if (rc != HTTPCLIENT_OK)
    {
        /* send header to webclient server failed. */
        return rc;
    }

    if (post_data && (data_len > 0))
    {
        httpClientWrite(session, post_data, data_len);

        /* resolve response data, get http status code */
        resp_status = httpClientHandleResponse(session);
        printf("post handle response(%d).", resp_status);
    }

    return resp_status;
}


/**
 * set receive and send data timeout.
 *
 * @param session http session
 * @param millisecond timeout millisecond
 *
 * @return 0: set timeout success
 */
int httpClientSetTimeout(struct httpClientSession *session, int millisecond)
{
    struct timeval timeout;
    int second = rt_tick_from_millisecond(millisecond) / 1000;

    assert(session);

    timeout.tv_sec = second;
    timeout.tv_usec = 0;

    /* set recv timeout option */
    setsockopt(session->socket, SOL_SOCKET, SO_RCVTIMEO,
               (void *) &timeout, sizeof(timeout));
    setsockopt(session->socket, SOL_SOCKET, SO_SNDTIMEO,
               (void *) &timeout, sizeof(timeout));

    return 0;
}

static int httpClientNextChunk(struct httpClientSession *session)
{
    char line[64];
    int length;

    assert(session);

    rt_memset(line, 0x00, sizeof(line));
    length = httpClientReadLine(session, line, sizeof(line));
    if (length > 0)
    {
        if (rt_strcmp(line, "\r") == 0)
        {
            length = httpClientReadLine(session, line, sizeof(line));
            if (length <= 0)
            {
                closesocket(session->socket);
                session->socket = -1;
                return length;
            }
        }
    }
    else
    {
        closesocket(session->socket);
        session->socket = -1;

        return length;
    }

    session->chunk_sz = strtol(line, RT_NULL, 16);
    session->chunk_offset = 0;

    if (session->chunk_sz == 0)
    {
        /* end of chunks */
        closesocket(session->socket);
        session->socket = -1;
        session->chunk_sz = -1;
    }

    return session->chunk_sz;
}

/**
 *  read data from http server.
 *
 * @param session http session
 * @param buffer read buffer
 * @param length the maximum of read buffer size
 *
 * @return <0: read data error
 *         =0: http server disconnect
 *         >0: successfully read data length
 */
int httpClientRead(struct httpClientSession *session, void *buffer, size_t length)
{
    int bytes_read = 0;
    int total_read = 0;
    int left;

    assert(session);

    /* get next chunk size is zero, client is already closed, return zero */
    if (session->chunk_sz < 0)
    {
        return 0;
    }

    if (session->socket < 0)
    {
        return -HTTPCLIENT_DISCONNECT;
    }

    if (length == 0)
    {
        return 0;
    }

    /* which is transfered as chunk mode */
    if (session->chunk_sz)
    {
        if ((int) length > (session->chunk_sz - session->chunk_offset))
        {
            length = session->chunk_sz - session->chunk_offset;
        }

        bytes_read = httpClientRecv(session, buffer, length, 0);
        if (bytes_read <= 0)
        {
            if (errno == EWOULDBLOCK || errno == EAGAIN)
            {
                /* recv timeout */
                return -HTTPCLIENT_TIMEOUT;
            }
            else
            {
                closesocket(session->socket);
                session->socket = -1;
                return 0;
            }
        }

        session->chunk_offset += bytes_read;
        if (session->chunk_offset >= session->chunk_sz)
        {
            httpClientNextChunk(session);
        }

        return bytes_read;
    }

    if (session->content_length > 0)
    {
        if (length > session->content_remainder)
        {
            length = session->content_remainder;
        }

        if (length == 0)
        {
            return 0;
        }
    }

    /*
     * Read until: there is an error, we've read "size" bytes or the remote
     * side has closed the connection.
     */
    left = length;
    do
    {
        bytes_read = httpClientRecv(session, (void *)((char *)buffer + total_read), left, 0);
        if (bytes_read <= 0)
        {
            printf("receive data error(%d).", bytes_read);

            if (total_read)
            {
                break;
            }
            else
            {
                if (errno == EWOULDBLOCK || errno == EAGAIN)
                {
                    /* recv timeout */
                    printf("receive data timeout.");
                    return -HTTPCLIENT_TIMEOUT;
                }
                else
                {
                    closesocket(session->socket);
                    session->socket = -1;
                    return 0;
                }
            }
        }

        left -= bytes_read;
        total_read += bytes_read;
    }
    while (left);

    if (session->content_length > 0)
    {
        session->content_remainder -= total_read;
    }

    return total_read;
}

/**
 *  write data to http server.
 *
 * @param session http session
 * @param buffer write buffer
 * @param length write buffer size
 *
 * @return <0: write data error
 *         =0: http server disconnect
 *         >0: successfully write data length
 */
int httpClientWrite(struct httpClientSession *session, const void *buffer, size_t length)
{
    int bytes_write = 0;
    int total_write = 0;
    int left = length;

    assert(session);

    if (session->socket < 0)
    {
        return -HTTPCLIENT_DISCONNECT;
    }

    if (length == 0)
    {
        return 0;
    }

    /* send all of data on the buffer. */
    do
    {
        bytes_write = httpClientSend(session, (void *)((char *)buffer + total_write), left, 0);
        if (bytes_write <= 0)
        {

            if (errno == EWOULDBLOCK || errno == EAGAIN)
            {
                /* send timeout */
                if (total_write)
                {
                    return total_write;
                }
                continue;
                /* TODO: whether return the TIMEOUT
                 * return -HTTPCLIENT_TIMEOUT; */
            }
            else
            {
                closesocket(session->socket);
                session->socket = -1;

                if (total_write == 0)
                {
                    return -HTTPCLIENT_DISCONNECT;
                }
                break;
            }
        }

        left -= bytes_write;
        total_write += bytes_write;
    }
    while (left);

    return total_write;
}

/* close session socket, free host and request url */
static int httpClientClean(struct httpClientSession *session)
{

    if (session->socket >= 0)
    {
        closesocket(session->socket);
        session->socket = -1;
    }
    if (session->host)
    {
        HTTP_FREE(session->host);
        session->host = RT_NULL;
    }

    if (session->req_url)
    {
        HTTP_FREE(session->req_url);
        session->req_url = RT_NULL;
    }

    session->content_length = -1;

    return 0;
}

/**
 * close a webclient client session.
 *
 * @param session http client session
 *
 * @return 0: close success
 */
int httpClientClose(struct httpClientSession *session)
{
    assert(session);

    httpClientClean(session);

    if (session->header && session->header->buffer)
    {
        HTTP_FREE(session->header->buffer);
    }

    if (session->header)
    {
        HTTP_FREE(session->header);
    }

    if (session)
    {
        HTTP_FREE(session);
        session = RT_NULL;
    }

    return 0;
}

/**
 * get wenclient request response data.
 *
 * @param session wenclient session
 * @param response response buffer address
 * @param resp_len response buffer length
 *
 * @return response data size
 */
int httpClientResponse(struct httpClientSession *session, void **response, size_t *resp_len)
{
    unsigned char *buf_ptr;
    unsigned char *response_buf = 0;
    int length, total_read = 0;

    assert(session);
    assert(response);

    /* initialize response */
    *response = RT_NULL;

    /* not content length field kind */
    if (session->content_length < 0)
    {
        size_t result_sz;

        total_read = 0;
        while (1)
        {
            unsigned char *new_resp = RT_NULL;

            result_sz = total_read + HTTPCLIENT_RESPONSE_BUFSZ;
            new_resp = HTTP_REALLOC(response_buf, result_sz + 1);
            if (new_resp == RT_NULL)
            {
                printf("no memory for realloc new response buffer!");
                break;
            }

            response_buf = new_resp;
            buf_ptr = (unsigned char *) response_buf + total_read;

            /* read result */
            length = httpClientRead(session, buf_ptr, result_sz - total_read);
            if (length <= 0)
                break;

            total_read += length;
        }
    }
    else
    {
        int result_sz;

        result_sz = session->content_length;
        response_buf = HTTP_CALLOC(1, result_sz + 1);
        if (response_buf == RT_NULL)
        {
            return -HTTPCLIENT_NOMEM;
        }

        buf_ptr = (unsigned char *) response_buf;
        for (total_read = 0; total_read < result_sz;)
        {
            length = httpClientRead(session, buf_ptr, result_sz - total_read);
            if (length <= 0)
                break;

            buf_ptr += length;
            total_read += length;
        }
    }

    if ((total_read == 0) && (response_buf != 0))
    {
        HTTP_FREE(response_buf);
        response_buf = RT_NULL;
    }

    if (response_buf)
    {
        *response = (void *)response_buf;
        *(response_buf + total_read) = '\0';
        *resp_len = total_read;
    }

    return total_read;
}

/**
 * add request(GET/POST) header data.
 *
 * @param request_header add request buffer address
 * @param fmt fields format
 *
 * @return <=0: add header failed
 *          >0: add header data size
 */

int httpClientRequestHeaderAdd(char **request_header, const char *fmt, ...)
{
    rt_int32_t length, header_length;
    char *header;
    va_list args;

    assert(request_header);

    if (*request_header == RT_NULL)
    {
        header = rt_calloc(1, HTTPCLIENT_HEADER_BUFSZ);
        if (header == RT_NULL)
        {
            printf("No memory for webclient request header add.");
            return RT_NULL;
        }
        *request_header = header;
    }
    else
    {
        header = *request_header;
    }

    va_start(args, fmt);
    header_length = rt_strlen(header);
    length = rt_vsnprintf(header + header_length, HTTPCLIENT_HEADER_BUFSZ - header_length, fmt, args);
    if (length < 0)
    {
        printf("add request header data failed, return length(%d) error.", length);
        return -HTTPCLIENT_ERROR;
    }
    va_end(args);

    /* check header size */
    if (rt_strlen(header) >= HTTPCLIENT_HEADER_BUFSZ)
    {
        printf("not enough request header data size(%d)!", HTTPCLIENT_HEADER_BUFSZ);
        return -HTTPCLIENT_ERROR;
    }

    return length;
}

/**
 *  send request(GET/POST) to server and get response data.
 *
 * @param URI input server address
 * @param header send header data
 *             = NULL: use default header data, must be GET request
 *            != NULL: user custom header data, GET or POST request
 * @param post_data data sent to the server
 *             = NULL: it is GET request
 *            != NULL: it is POST request
 * @param data_len send data length
 * @param response response buffer address
 * @param resp_len response buffer length
 *
 * @return <0: request failed
 *        >=0: response buffer size
 */
int httpClientRequest(const char *URI, const char *header, const void *post_data, size_t data_len, void **response, size_t *resp_len)
{
    struct httpClientSession *session = RT_NULL;
    int rc = HTTPCLIENT_OK;
    int totle_length = 0;

    assert(URI);

    if (post_data == RT_NULL && response == RT_NULL)
    {
        printf("request get failed, get response data cannot be empty.");
        return -HTTPCLIENT_ERROR;
    }

    if ((post_data != RT_NULL) && (data_len == 0))
    {
        printf("input post data length failed");
        return -HTTPCLIENT_ERROR;
    }

    if ((response != RT_NULL && resp_len == RT_NULL) ||
        (response == RT_NULL && resp_len != RT_NULL))
    {
        printf("input response data or length failed");
        return -HTTPCLIENT_ERROR;
    }

    if (post_data == RT_NULL)
    {
        /* send get request */
        session = httpClientSessionCreate(HTTPCLIENT_HEADER_BUFSZ);
        if (session == RT_NULL)
        {
            rc = -HTTPCLIENT_NOMEM;
            goto __exit;
        }

        if (header != RT_NULL)
        {
            char *header_str, *header_ptr;
            int header_line_length;

            for(header_str = (char *)header; (header_ptr = rt_strstr(header_str, "\r\n")) != RT_NULL; )
            {
                header_line_length = header_ptr + rt_strlen("\r\n") - header_str;
                httpClientHeaderFieldsAdd(session, "%.*s", header_line_length, header_str);
                header_str += header_line_length;
            }
        }

        if (httpClientGet(session, URI) != 200)
        {
            rc = -HTTPCLIENT_ERROR;
            goto __exit;
        }

        totle_length = httpClientResponse(session, response, resp_len);
        if (totle_length <= 0)
        {
            rc = -HTTPCLIENT_ERROR;
            goto __exit;
        }
    }
    else
    {
        /* send post request */
        session = httpClientSessionCreate(HTTPCLIENT_HEADER_BUFSZ);
        if (session == RT_NULL)
        {
            rc = -HTTPCLIENT_NOMEM;
            goto __exit;
        }

        if (header != RT_NULL)
        {
            char *header_str, *header_ptr;
            int header_line_length;

            for(header_str = (char *)header; (header_ptr = rt_strstr(header_str, "\r\n")) != RT_NULL; )
            {
                header_line_length = header_ptr + rt_strlen("\r\n") - header_str;
                httpClientHeaderFieldsAdd(session, "%.*s", header_line_length, header_str);
                header_str += header_line_length;
            }
        }

        if (rt_strstr(session->header->buffer, "Content-Length") == RT_NULL)
        {
            httpClientHeaderFieldsAdd(session, "Content-Length: %d\r\n", rt_strlen(post_data));
        }

        if (rt_strstr(session->header->buffer, "Content-Type") == RT_NULL)
        {
            httpClientHeaderFieldsAdd(session, "Content-Type: application/octet-stream\r\n");
        }

        if (httpClientPost(session, URI, post_data, data_len) != 200)
        {
            rc = -HTTPCLIENT_ERROR;
            goto __exit;
        }

        totle_length = httpClientResponse(session, response, resp_len);
        if (totle_length <= 0)
        {
            rc = -HTTPCLIENT_ERROR;
            goto __exit;
        }
    }

__exit:
    if (session)
    {
        httpClientClose(session);
        session = RT_NULL;
    }

    if (rc < 0)
    {
        return rc;
    }

    return totle_length;
}


int httpClientGetFile(const char* URI, const char* filename)
{
    int fd = -1, rc = HTTPCLIENT_OK;
    size_t offset;
    int length, total_length = 0;
    unsigned char *ptr = RT_NULL;
    struct httpClientSession* session = RT_NULL;
    int resp_status = 0;

    session = httpClientSessionCreate(HTTPCLIENT_HEADER_BUFSZ);
    if(session == RT_NULL)
    {
        rc = -HTTPCLIENT_NOMEM;
        goto __exit;
    }

    if ((resp_status = httpClientGet(session, URI)) != 200)
    {
        printf("get file failed, wrong response: %d (-0x%X).", resp_status, resp_status);
        rc = -HTTPCLIENT_ERROR;
        goto __exit;
    }

    fd = open(filename, O_WRONLY | O_CREAT | O_TRUNC, 0);
    if (fd < 0)
    {
        printf("get file failed, open file(%s) error.", filename);
        rc = -HTTPCLIENT_ERROR;
        goto __exit;
    }

    ptr = (unsigned char *) HTTP_MALLOC(HTTPCLIENT_RESPONSE_BUFSZ);
    if (ptr == RT_NULL)
    {
        printf("get file failed, no memory for response buffer.");
        rc = -HTTPCLIENT_NOMEM;
        goto __exit;
    }

    if (session->content_length < 0)
    {
        while (1)
        {
            length = httpClientRead(session, ptr, HTTPCLIENT_RESPONSE_BUFSZ);
            if (length > 0)
            {
                write(fd, ptr, length);
                total_length += length;
                printf(">");
            }
            else
            {
                break;
            }
        }
    }
    else
    {
        for (offset = 0; offset < (size_t) session->content_length;)
        {
            length = httpClientRead(session, ptr,
                    session->content_length - offset > HTTPCLIENT_RESPONSE_BUFSZ ?
                            HTTPCLIENT_RESPONSE_BUFSZ : session->content_length - offset);

            if (length > 0)
            {
                write(fd, ptr, length);
                total_length += length;
                printf(">");
            }
            else
            {
                break;
            }

            offset += length;
        }
    }

    if (total_length)
    {
        printf("save %d bytes.", total_length);
    }

__exit:
    if (fd >= 0)
    {
        close(fd);
    }

    if (session != RT_NULL)
    {
        httpClientClose(session);
    }

    if (ptr != RT_NULL)
    {
        HTTP_FREE(ptr);
    }

    return rc;
}


int httpGet(int argc, char** argv)
{
    if (argc != 3)
    {
        printf("Please using: wget <URI> <filename>\n");
        return -1;
    }

    httpClientGetFile(argv[1], argv[2]);
    return 0;
}

#ifdef __RT_THREAD_H__
MSH_CMD_EXPORT(httpGet, Get file by URI: httpGet <URI> <filename>.);
#endif /* FINSH_USING_MSH */