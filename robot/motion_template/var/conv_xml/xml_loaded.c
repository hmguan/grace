#include <stdlib.h>
#include <stdio.h>
#include <memory.h>
#include "xml_loaded.h"
#include "posix_string.h"
#include "posix_ifos.h"

const char *get_first_text_value(xmlNodePtr node) {
    static char value[128];
    xmlNodePtr next;

    if (!node) return NULL;

    next = node;
    while (next) {
        if (XML_ELEMENT_NODE == next->type || XML_TEXT_NODE == next->type) {
            xmlChar *sz = xmlNodeGetContent(next);
            if (sz) {
                posix__strcpy(value, cchof(value), (const char *)sz);
                xmlFree(sz);
                return value;
            }
        }
        next = next->children;
    }
    return NULL;
}

double covert_first_text_value_d(xmlNodePtr node) {
    const char *value = get_first_text_value(node);
    if (value) {
        return strtod(value, NULL);
    }

    return 0.0;
}

int covert_first_text_value_i(xmlNodePtr node) {
    return (int) covert_first_text_value_l(node);
}

long covert_first_text_value_l(xmlNodePtr node) {
    const char *value = get_first_text_value(node);
    if (value) {
        return strtol(value, NULL, 10);
    }

    return 0;
}

uint64_t covert_first_text_value_ull(xmlNodePtr node) {
    const char *value = get_first_text_value(node);
    if (value) {
        return strtoull(value, NULL, 10);
    }
    return 0;
}

void covert_first_text_value_s(xmlNodePtr node, char *target, int cch) {
    const char *value = get_first_text_value(node);
    if (value && target && cch > 0) {
        posix__strcpy(target, cch, value);
    }
}

const char *get_first_text_properties(xmlAttr *attr) {
    static char value[128];
    xmlNode *properties;
    xmlChar *sz;

    if (!attr) return NULL;

    properties = attr->children;
    while (properties) {
        if (XML_TEXT_NODE == properties->type) {
            sz = xmlNodeGetContent(properties);
            if (sz) {
                posix__strcpy(value, cchof(value), (const char *)sz);
                xmlFree(sz);
                return value;
            }
        }
        properties = properties->next;
    }

    return NULL;
}

int xml__read_data(const char *subdir, const char *file, const char *rootname, int( *handler)(xmlNodePtr element)) {
    
    char xml_path[256];
    xmlDocPtr doc;
    xmlNodePtr root = NULL;
    xmlNodePtr element = NULL;
    int retval;

    if (!file || !rootname || !handler) return -1;

    retval = -1;

#if _WIN32
    const char *pedir;
    
    pedir = posix__getpedir();
    if (!pedir) {
        return -1;
    }
    
    if (subdir){
        posix__sprintf(xml_path, cchof(xml_path), "%s\\etc\\%s\\%s", pedir, subdir, file);
    }else{
        posix__sprintf(xml_path, cchof(xml_path), "%s\\etc\\%s", pedir, file);
    }
#else
    if (subdir){
        posix__sprintf(xml_path, cchof(xml_path), "/etc/agv/%s/%s", subdir, file);
    }else{
        posix__sprintf(xml_path, cchof(xml_path), "/etc/agv/%s", file);
    }
#endif
    
    doc = xmlReadFile(xml_path, "GB2312", XML_PARSE_NOBLANKS);
    if (!doc) {
        doc = xmlReadFile(xml_path, "UTF-8", XML_PARSE_NOBLANKS);
        if (!doc) {
            return -1;
        }
    }

    if (NULL != (root = xmlDocGetRootElement(doc))) {
        if (0 == xmlStrcmp(BAD_CAST rootname, root->name)) {
            element = root->xmlChildrenNode;
            if (element) {
                retval = handler(element);
            }
        }
    }

    xmlFreeDoc(doc);
    return retval;
}

int xml__check_candevice_head(xmlNode *node, var__can_device_t *candev) {
    if (!candev || !node) {
        return -1;
    }

    if (0 == xmlStrcmp(BAD_CAST"can", node->name)) {
        candev->canbus_ = covert_first_text_value_i(node);
    } else if (0 == xmlStrcmp(BAD_CAST"port", node->name)) {
        candev->canport_ = covert_first_text_value_i(node);
    } else if (0 == xmlStrcmp(BAD_CAST"node", node->name)) {
        candev->cannode_ = covert_first_text_value_i(node);
    } else if (0 == xmlStrcmp(BAD_CAST"latency", node->name)) {
        candev->latency_ = covert_first_text_value_ull(node);
    } else if (0 == xmlStrcmp(BAD_CAST"merge", node->name)) {
        candev->merge_ = covert_first_text_value_i(node);
    } else if (0 == xmlStrcmp(BAD_CAST"pdocnt", node->name)) {
        candev->pdocnt_ = covert_first_text_value_i(node);
    } else {
        return -1;
    }

    return 0;
}