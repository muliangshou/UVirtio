/*
* Copyright (c) 2021 AIIT XUOS Lab
* XiUOS is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2.
* You may obtain a copy of Mulan PSL v2 at:
*        http://license.coscl.org.cn/MulanPSL2
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
* EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
* MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
* See the Mulan PSL v2 for more details.
*/

/**
 * @file ua_client.c
 * @brief Client for OpcUa function
 * @version 1.0
 * @author AIIT XUOS Lab
 * @date 2021.11.11
 */

#include <stdlib.h>
#include "open62541.h"
#include "ua_api.h"

#define UA_RESPONSE_TIMEOUT 10000

const char *opc_server_url = {"opc.tcp://192.168.250.2:4840"};

#ifdef UA_ENABLE_SUBSCRIPTIONS
static void UaAnswerChangedHandler(UA_Client* client, UA_UInt32 subId, void* subContext,
                                     UA_UInt32 monId, void* monContext, UA_DataValue* value)
{
    ua_notice("Answer changed!\n");
}
#endif

static UA_StatusCode UaShowNodeIdIterate(UA_NodeId childId, UA_Boolean isInverse, UA_NodeId referenceTypeId, void* handle)
{
    if(isInverse)
    {
        return UA_STATUSCODE_GOOD;
    }

    UA_NodeId* parent = (UA_NodeId*)handle;
    ua_notice("%d, %d --- %d ---> NodeId %d, %d\n",
               parent->namespaceIndex, parent->identifier.numeric,
               referenceTypeId.identifier.numeric, childId.namespaceIndex,
               childId.identifier.numeric);
    return UA_STATUSCODE_GOOD;
}

int UaGetEndPoints(UA_Client* client)
{
    /* Listing endpoints */
    UA_EndpointDescription* endpointArray = NULL;
    size_t endpointArraySize = 0;
    UA_StatusCode ret = UA_Client_getEndpoints(client, opc_server_url,
                                                  &endpointArraySize, &endpointArray);

    if(ret != UA_STATUSCODE_GOOD)
    {
        UA_Array_delete(endpointArray, endpointArraySize, &UA_TYPES[UA_TYPES_ENDPOINTDESCRIPTION]);
        return EXIT_FAILURE;
    }

    ua_print("%i endpoints found\n", (int)endpointArraySize);

    for(size_t i=0; i<endpointArraySize; i++)
    {
        ua_print("URL of endpoint %i is %.*s\n", (int)i,
                 (int)endpointArray[i].endpointUrl.length,
                 endpointArray[i].endpointUrl.data);
    }

    UA_Array_delete(endpointArray,endpointArraySize, &UA_TYPES[UA_TYPES_ENDPOINTDESCRIPTION]);
    return EXIT_SUCCESS;
}

static void UaShowNodeValue(UA_Variant* val)
{
    if(val->type == &UA_TYPES[UA_TYPES_LOCALIZEDTEXT])
    {
        UA_LocalizedText* ptr = (UA_LocalizedText*)val->data;
        ua_notice("%.*s (Text)\n", ptr->text.length, ptr->text.data);
    }
    else if(val->type == &UA_TYPES[UA_TYPES_UINT32])
    {
        UA_UInt32* ptr = (UA_UInt32*)val->data;
        ua_notice("%d (UInt32)\n", *ptr);
    }
    else if(val->type == &UA_TYPES[UA_TYPES_BOOLEAN])
    {
        UA_Boolean* ptr = (UA_Boolean*)val->data;
        ua_notice("%i (BOOL)\n", *ptr);
    }
    else if(val->type == &UA_TYPES[UA_TYPES_INT32])
    {
        UA_Int32* ptr = (UA_Int32*)val->data;
        ua_notice("%d (Int32)\n", *ptr);
    }
    else if(val->type == &UA_TYPES[UA_TYPES_INT16])
    {
        UA_Int16* ptr = (UA_Int16*)val->data;
        ua_notice("%d (Int16)\n", *ptr);
    }
    else if(val->type == &UA_TYPES[UA_TYPES_FLOAT])
    {
        UA_Float* ptr = (UA_Float*)val->data;
        printf("%f (Float)\n", *ptr);
    }
    else if(val->type == &UA_TYPES[UA_TYPES_DOUBLE])
    {
        UA_Double* ptr = (UA_Double*)val->data;
        printf("%f (Double)\n", *ptr);
    }
    else if(val->type == &UA_TYPES[UA_TYPES_STRING])
    {
        UA_String* ptr = (UA_String*)val->data;
        ua_notice("%*.s (String)\n", ptr->length, ptr->data);
    }
    else if(val->type == &UA_TYPES[UA_TYPES_DATETIME])
    {
        UA_DateTime* ptr = (UA_DateTime*)val->data;
        UA_DateTimeStruct dts = UA_DateTime_toStruct(*ptr);
        ua_notice("%d-%d-%d %d:%d:%d.%03d (Time)\n",
                   dts.day, dts.month, dts.year, dts.hour, dts.min, dts.sec, dts.milliSec);
    }
}

char *UaGetNodeIdString(UA_NodeId *node_id)
{
    static char nodeid_str[UA_NODE_LEN] = {0};

    switch(node_id->identifierType)
    {
        case UA_NODEIDTYPE_NUMERIC:
            snprintf(nodeid_str, UA_NODE_LEN, "n%d,%d", node_id->namespaceIndex, node_id->identifier.numeric);
            break;
        case UA_NODEIDTYPE_STRING:
            snprintf(nodeid_str, UA_NODE_LEN, "n%d,%.*s", node_id->namespaceIndex, node_id->identifier.string.length,
                node_id->identifier.string.data);
            break;
        case UA_NODEIDTYPE_BYTESTRING:
            snprintf(nodeid_str, UA_NODE_LEN, "n%d,%s", node_id->namespaceIndex, node_id->identifier.byteString.data);
            break;
        default:
            break;
    }
    return nodeid_str;
}

static void UaShowNodeId(UA_NodeId *node_id)
{
    switch(node_id->identifierType)
    {
        case UA_NODEIDTYPE_NUMERIC:
            ua_notice(" NodeID n%d,%d ", node_id->namespaceIndex, node_id->identifier.numeric);
            break;
        case UA_NODEIDTYPE_STRING:
            ua_notice(" NodeID n%d,%.*s ", node_id->namespaceIndex, node_id->identifier.string.length,
                node_id->identifier.string.data);
            break;
        case UA_NODEIDTYPE_BYTESTRING:
            ua_notice(" NodeID n%d,%s ", node_id->namespaceIndex, node_id->identifier.byteString.data);
            break;
        default:
            break;
    }
}

static void UaShowObject(UA_BrowseResponse* res)
{
    ua_notice("%-9s %-16s %-16s %-16s\n", "NAMESPACE", "NODEID", "BROWSE NAME", "DISPLAY NAME");

    for(size_t i = 0; i < res->resultsSize; ++i)
    {
        for(size_t j = 0; j < res->results[i].referencesSize; ++j)
        {
            UA_ReferenceDescription* ref = &(res->results[i].references[j]);

            if(ref->nodeId.nodeId.identifierType == UA_NODEIDTYPE_NUMERIC)
            {
                ua_notice("%-9d %-16d %-16.*s %-16.*s\n", ref->nodeId.nodeId.namespaceIndex,
                           ref->nodeId.nodeId.identifier.numeric, (int)ref->browseName.name.length,
                           ref->browseName.name.data, (int)ref->displayName.text.length,
                           ref->displayName.text.data);
            }
            else if(ref->nodeId.nodeId.identifierType == UA_NODEIDTYPE_STRING)
            {
                ua_notice("%-9d %-16.*s %-16.*s %-16.*s\n", ref->nodeId.nodeId.namespaceIndex,
                           (int)ref->nodeId.nodeId.identifier.string.length,
                           ref->nodeId.nodeId.identifier.string.data,
                           (int)ref->browseName.name.length, ref->browseName.name.data,
                           (int)ref->displayName.text.length, ref->displayName.text.data);
            }

            /* TODO: distinguish further types */
        }
    }

    ua_notice("\n");
}

UA_StatusCode UaReadArrayValue(UA_Client* client, int array_size, UA_ReadValueId* array)
{
    UA_ReadRequest request;
    UA_ReadRequest_init(&request);
    request.nodesToRead = array;
    request.nodesToReadSize = array_size;
    UA_ReadResponse response = UA_Client_Service_read(client, request);

    if((response.responseHeader.serviceResult != UA_STATUSCODE_GOOD)
            || (response.resultsSize != array_size))
    {
        UA_ReadResponse_clear(&response);
        ua_notice("ua: [%s] read failed 0x%x\n", __func__,
                   response.responseHeader.serviceResult);
        return UA_STATUSCODE_BADUNEXPECTEDERROR;
    }

    UA_StatusCode* arr_ret = malloc(array_size * sizeof(UA_StatusCode));
    if(arr_ret == NULL)
    {
        ua_error("ua: [%s] malloc %d failed!\n", __func__, array_size * sizeof(UA_StatusCode));
        return UA_STATUSCODE_BADOUTOFMEMORY;
    }

    for(int i = 0; i < array_size; ++i)
    {
        if((response.results[i].status == UA_STATUSCODE_GOOD)
                && (response.results[i].hasValue))
        {
            ua_notice("node %s: ", UaGetNodeIdString(&array[i].nodeId));
            UaShowNodeValue(&response.results[i].value);
        }
    }
    ua_notice("\n");

    free(arr_ret);
    UA_ReadResponse_clear(&response);
    return UA_STATUSCODE_GOOD;
}

void UaBrowserNodeId(UA_Client* client, UA_NodeId id)
{
    UA_BrowseRequest ua_req;
    UA_BrowseResponse ua_resp;

    /* Browse some objects */
    ua_notice("Browsing nodes in objects folder:\n");

    UA_BrowseRequest_init(&ua_req);

    ua_req.requestedMaxReferencesPerNode = 0;
    ua_req.nodesToBrowse = UA_BrowseDescription_new();
    ua_req.nodesToBrowseSize = 1;
    ua_req.nodesToBrowse[0].nodeId = id; /* browse objects folder */
    ua_req.nodesToBrowse[0].resultMask = UA_BROWSERESULTMASK_ALL; /* return everything */

    ua_resp = UA_Client_Service_browse(client, ua_req);

    UaShowObject(&ua_resp);

    UA_BrowseResponse_clear(&ua_resp);
}


int UaGetNodeIdArray(UA_Client* client, UA_NodeId id, int array_size, int *id_array)
{
    int i, j;
    int array_cnt = 0;// return array number
    UA_BrowseRequest ua_req;
    UA_BrowseResponse ua_resp;

    /* Browse some objects */
    ua_notice("Browsing nodes in objects folder:\n");

    UA_BrowseRequest_init(&ua_req);

    ua_req.requestedMaxReferencesPerNode = 0;
    ua_req.nodesToBrowse = UA_BrowseDescription_new();
    ua_req.nodesToBrowseSize = 1;
    ua_req.nodesToBrowse[0].nodeId = id; /* browse objects folder */
    ua_req.nodesToBrowse[0].resultMask = UA_BROWSERESULTMASK_ALL; /* return everything */

    ua_resp = UA_Client_Service_browse(client, ua_req);

    for(i = 0; i < ua_resp.resultsSize; ++i)
    {
        for(j = 0; j < ua_resp.results[i].referencesSize; ++j)
        {
            UA_ReferenceDescription* ref = &(ua_resp.results[i].references[j]);

            if(ref->nodeId.nodeId.identifierType == UA_NODEIDTYPE_NUMERIC)
            {
                *(id_array + array_cnt) = ref->nodeId.nodeId.identifier.numeric;
                array_cnt ++;
                if(array_cnt >= array_size)
                {
                    UA_BrowseResponse_clear(&ua_resp);
                    return array_cnt;
                }
            }
        }
    }

    UA_BrowseResponse_clear(&ua_resp);
    return array_cnt;
}

void UaBrowserNodes(UA_Client* client)
{
    UA_NodeId* parent = UA_NodeId_new();
    *parent = UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER);
    UA_Client_forEachChildNodeCall(client, UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER), UaShowNodeIdIterate, (void*) parent);
    UA_NodeId_delete(parent);
}

UA_UInt32 UaStartSubscription(UA_Client* client, UA_NodeId node_id)
{
    /* Create a subscription */
    UA_CreateSubscriptionRequest request = UA_CreateSubscriptionRequest_default();
    UA_CreateSubscriptionResponse response = UA_Client_Subscriptions_create(client, request,
                                                                            NULL, NULL, NULL);
    UA_UInt32 subId = response.subscriptionId;

    if(response.responseHeader.serviceResult == UA_STATUSCODE_GOOD)
    {
        ua_print("Create subscription succeeded, id %u\n", subId);
    }
    else
    {
        ua_print("Create subscription failed, id %u\n", response.responseHeader.serviceResult);
        return response.responseHeader.serviceResult;
    }

    UA_MonitoredItemCreateRequest monRequest =
        UA_MonitoredItemCreateRequest_default(node_id);
    UA_MonitoredItemCreateResult monResponse =
        UA_Client_MonitoredItems_createDataChange(client, response.subscriptionId,
                                                  UA_TIMESTAMPSTORETURN_BOTH,
                                                  monRequest, NULL, UaAnswerChangedHandler, NULL);

    if(monResponse.statusCode == UA_STATUSCODE_GOOD)
    {
        ua_print("Monitoring 'the.answer', id %u\n", monResponse.monitoredItemId);
    }
    else
    {
        ua_print("%s return 0x%x\n", __func__, monResponse.statusCode);
    }

    /* The first publish request should return the initial value of the variable */
    UA_Client_run_iterate(client, UA_RESPONSE_TIMEOUT);
    return subId;
}

void UaWriteNodeValue(UA_Client* client, UA_NodeId id, char* value)
{
    UA_Boolean bool_val;
    uint32_t integer_val;
    UA_WriteRequest wReq;
    UA_WriteRequest_init(&wReq);

    wReq.nodesToWrite = UA_WriteValue_new();
    wReq.nodesToWriteSize = 1;

    if(strncmp(value, "1b", 2) == 0)
    {
        wReq.nodesToWrite[0].value.value.type = &UA_TYPES[UA_TYPES_BOOLEAN];
        bool_val = 1;
        wReq.nodesToWrite[0].value.value.data = &bool_val;
    }
    else if(strncmp(value, "0b", 2) == 0)
    {
        wReq.nodesToWrite[0].value.value.type = &UA_TYPES[UA_TYPES_BOOLEAN];
        bool_val = 0;
        wReq.nodesToWrite[0].value.value.data = &bool_val;
    }
    else
    {
        wReq.nodesToWrite[0].value.value.type = &UA_TYPES[UA_TYPES_INT16];
        sscanf(value, "%d", &integer_val);
        wReq.nodesToWrite[0].value.value.data = &integer_val;
    }

    wReq.nodesToWrite[0].nodeId = id;
    wReq.nodesToWrite[0].attributeId = UA_ATTRIBUTEID_VALUE;
    wReq.nodesToWrite[0].value.hasValue = true;
    wReq.nodesToWrite[0].value.value.storageType = UA_VARIANT_DATA_NODELETE; /* do not free the integer on deletion */
    UA_WriteResponse wResp = UA_Client_Service_write(client, wReq);

    if(wResp.responseHeader.serviceResult == UA_STATUSCODE_GOOD)
    {
        ua_notice("write new value is: %s\n", value);
    }

    UA_WriteRequest_clear(&wReq);
    UA_WriteResponse_clear(&wResp);
}

/* Read attribute */
void UaReadNodeValue(UA_Client* client, UA_NodeId id, UA_Int32 *value)
{
    UA_Variant* val = UA_Variant_new();
    UA_StatusCode ret = UA_Client_readValueAttribute(client, id, val);

    if(ret == UA_STATUSCODE_GOOD)
    {
        UaShowNodeValue(val);
        if(UA_Variant_isScalar(val))
        {
            if(val->type == &UA_TYPES[UA_TYPES_BOOLEAN])
            {
                *value = *(UA_Boolean *)val->data;
            }
            else if(val->type == &UA_TYPES[UA_TYPES_INT32])
            {
                *value = *(UA_Int32 *)val->data;
            }
            else if(val->type == &UA_TYPES[UA_TYPES_INT16])
            {
                *value = *(UA_Int16 *)val->data;
            }
        }
    }

    UA_Variant_delete(val);
}

void UaCallRemote(UA_Client* client)
{
    /* Call a remote method */
    UA_Variant input;
    UA_String argString = UA_STRING("Hello Server");
    UA_Variant_init(&input);
    UA_Variant_setScalarCopy(&input, &argString, &UA_TYPES[UA_TYPES_STRING]);
    size_t outputSize;
    UA_Variant* output;
    UA_StatusCode ret = UA_Client_call(client, UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),
                                          UA_NODEID_NUMERIC(1, 62541), 1, &input, &outputSize, &output);

    if(ret == UA_STATUSCODE_GOOD)
    {
        ua_print("Method call was successful, and %lu returned values available.\n",
                 (unsigned long)outputSize);
        UA_Array_delete(output, outputSize, &UA_TYPES[UA_TYPES_VARIANT]);
    }
    else
    {
        ua_print("Method call was unsuccessful, and %x returned values available.\n", ret);
    }

    UA_Variant_clear(&input);
}


void UaAddNodes(UA_Client* client)
{
    /* Add new nodes*/
    /* New ReferenceType */
    UA_NodeId ref_id;
    UA_ReferenceTypeAttributes ref_attr = UA_ReferenceTypeAttributes_default;
    ref_attr.displayName = UA_LOCALIZEDTEXT("en-US", "NewReference");
    ref_attr.description = UA_LOCALIZEDTEXT("en-US", "References something that might or might not exist");
    ref_attr.inverseName = UA_LOCALIZEDTEXT("en-US", "IsNewlyReferencedBy");
    UA_StatusCode ret = UA_Client_addReferenceTypeNode(client,
                                                          UA_NODEID_NUMERIC(1, 12133),
                                                          UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
                                                          UA_NODEID_NUMERIC(0, UA_NS0ID_HASSUBTYPE),
                                                          UA_QUALIFIEDNAME(1, "NewReference"),
                                                          ref_attr, &ref_id);

    if(ret == UA_STATUSCODE_GOOD)
    {
        ua_print("Created 'NewReference' with numeric NodeID %u\n", ref_id.identifier.numeric);
    }

    /* New ObjectType */
    UA_NodeId objt_id;
    UA_ObjectTypeAttributes objt_attr = UA_ObjectTypeAttributes_default;
    objt_attr.displayName = UA_LOCALIZEDTEXT("en-US", "TheNewObjectType");
    objt_attr.description = UA_LOCALIZEDTEXT("en-US", "Put innovative description here");
    ret = UA_Client_addObjectTypeNode(client,
                                         UA_NODEID_NUMERIC(1, 12134),
                                         UA_NODEID_NUMERIC(0, UA_NS0ID_BASEOBJECTTYPE),
                                         UA_NODEID_NUMERIC(0, UA_NS0ID_HASSUBTYPE),
                                         UA_QUALIFIEDNAME(1, "NewObjectType"),
                                         objt_attr, &objt_id);

    if(ret == UA_STATUSCODE_GOOD)
    {
        ua_print("Created 'NewObjectType' with numeric NodeID %u\n", objt_id.identifier.numeric);
    }

    /* New Object */
    UA_NodeId obj_id;
    UA_ObjectAttributes obj_attr = UA_ObjectAttributes_default;
    obj_attr.displayName = UA_LOCALIZEDTEXT("en-US", "TheNewGreatNode");
    obj_attr.description = UA_LOCALIZEDTEXT("de-DE", "Hier koennte Ihre Webung stehen!");
    ret = UA_Client_addObjectNode(client,
                                     UA_NODEID_NUMERIC(1, 0),
                                     UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),
                                     UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
                                     UA_QUALIFIEDNAME(1, "TheGreatNode"),
                                     UA_NODEID_NUMERIC(1, 12134),
                                     obj_attr, &obj_id);

    if(ret == UA_STATUSCODE_GOOD)
    {
        ua_print("Created 'NewObject' with numeric NodeID %u\n", obj_id.identifier.numeric);
    }

    /* New Integer Variable */
    UA_NodeId var_id;
    UA_VariableAttributes var_attr = UA_VariableAttributes_default;
    var_attr.displayName = UA_LOCALIZEDTEXT("en-US", "TheNewVariableNode");
    var_attr.description =
        UA_LOCALIZEDTEXT("en-US", "This integer is just amazing - it has digits and everything.");
    UA_Int32 int_value = 1234;
    /* This does not copy the value */
    UA_Variant_setScalar(&var_attr.value, &int_value, &UA_TYPES[UA_TYPES_INT32]);
    var_attr.dataType = UA_TYPES[UA_TYPES_INT32].typeId;
    ret = UA_Client_addVariableNode(client,
                                       UA_NODEID_NUMERIC(1, 0), // Assign new/random NodeID
                                       UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),
                                       UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
                                       UA_QUALIFIEDNAME(0, "VariableNode"),
                                       UA_NODEID_NULL, // no variable type
                                       var_attr, &var_id);

    if(ret == UA_STATUSCODE_GOOD)
    {
        ua_print("Created 'NewVariable' with numeric NodeID %u\n", var_id.identifier.numeric);
    }
}

void UaGetServerTime(UA_Client* client)
{
    UA_Variant value;
    UA_Variant_init(&value);

    const UA_NodeId nodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_SERVER_SERVERSTATUS_CURRENTTIME);
    UA_StatusCode ret = UA_Client_readValueAttribute(client, nodeId, &value);

    if(ret == UA_STATUSCODE_GOOD && UA_Variant_hasScalarType(&value, &UA_TYPES[UA_TYPES_DATETIME]))
    {
        UA_DateTime raw_date = *(UA_DateTime*) value.data;
        UA_DateTimeStruct dts = UA_DateTime_toStruct(raw_date);
        ua_notice("date is: %d-%d-%d %d:%d:%d.%03d\n",
                    dts.day, dts.month, dts.year, dts.hour, dts.min, dts.sec, dts.milliSec);
    }

    /* Clean up */
    UA_Variant_clear(&value);
}

