#include "wsdd.nsmap"
#include "soapH.h"

int __wsdd__Probe(struct soap *soap,
                  struct wsdd__ProbeType *probe,
                  struct wsdd__ProbeMatchesType *matches)
{
    printf("🔍 Probe received!\n");

    matches->__sizeProbeMatch = 1;
    matches->ProbeMatch = soap_new_wsdd__ProbeMatchType(soap, 1);

    struct wsdd__ProbeMatchType *pm = matches->ProbeMatch;

    pm->wsa__EndpointReference.Address = soap_strdup(soap, "urn:uuid:JetsonONVIF-001");
    pm->Types = soap_strdup(soap, "dn:NetworkVideoTransmitter");
    pm->Scopes = soap_strdup(soap, "onvif://www.onvif.org/type/video_encoder");
    pm->XAddrs = soap_strdup(soap, "http://192.168.33.13:1000/onvif/device_service");  // 포트와 주소 맞게 조정
    pm->MetadataVersion = 1;

    return SOAP_OK;
}