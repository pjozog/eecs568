#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <inttypes.h>
#include <stdio.h>

#include "image_source.h"
#include "url_parser.h"

image_source_t *image_source_open(const char *url)
{
    image_source_t *isrc = NULL;

    // get feature key/value pairs
    url_parser_t *urlp = url_parser_create(url);
    if (urlp == NULL) // bad URL format
        return NULL;

    const char *protocol = url_parser_get_protocol(urlp);
    const char *location = url_parser_get_location(urlp);

    // open device
    if (!strcmp(protocol, "v4l2://"))
        isrc = image_source_v4l2_open(location);
    else if (!strcmp(protocol, "dc1394://")) {
        isrc = image_source_dc1394_open(urlp);
    } else if (!strcmp(protocol, "islog://")) {
        isrc = image_source_islog_open(urlp);
    }

    // handle parameters
    if (isrc != NULL) {
        int found[url_parser_num_parameters(urlp)];

        for (int param_idx = 0; param_idx < url_parser_num_parameters(urlp); param_idx++) {
            const char *key = url_parser_get_parameter_name(urlp, param_idx);
            const char *value = url_parser_get_parameter_value(urlp, param_idx);

            if (!strcmp(key, "fidx")) {
                printf("image_source_dc1394.c: set feature %30s = %15s\n", key, value);
                int fidx = atoi(url_parser_get_parameter(urlp, "fidx", "0"));
                isrc->set_format(isrc, fidx);
                found[param_idx] = 1;
                continue;
            }

            if (!strcmp(key, "format")) {
                printf("image_source_dc1394.c: set feature %30s = %15s\n", key, value);
                isrc->set_named_format(isrc, value);
                found[param_idx] = 1;
                continue;
            }

            // pass through a device-specific parameter.
            for (int feature_idx = 0; feature_idx < isrc->num_features(isrc); feature_idx++) {

                if (!strcmp(isrc->get_feature_name(isrc, feature_idx), key)) {
                    char *endptr = NULL;
                    double dv = strtod(value, &endptr);
                    if (endptr != value + strlen(value)) {
                        printf("Parameter for key '%s' is invalid. Must be a number.\n",
                               isrc->get_feature_name(isrc, feature_idx));
                        goto cleanup;
                    }

                    int res = isrc->set_feature_value(isrc, feature_idx, dv);
                    if (res != 0)
                        printf("Error setting feature: key %s value %s, error code %d\n",
                               key, value, res);

                    double setvalue = isrc->get_feature_value(isrc, feature_idx);
                    printf("image_source_dc1394.c: set feature %30s = %15s. Actually set to %8.3f\n", key, value, setvalue);

                    found[param_idx] = 1;
                    break;
                }
            }
        }

        for (int param_idx = 0; param_idx < url_parser_num_parameters(urlp); param_idx++) {
            if (found[param_idx] != 1) {
                const char *key = url_parser_get_parameter_name(urlp, param_idx);
                const char *value = url_parser_get_parameter_value(urlp, param_idx);

                printf("Parameter not found. Key: %s Value: %s\n", key, value);
            }
        }
    }

cleanup:
    url_parser_destroy(urlp);

    return isrc;
}

char** image_source_enumerate()
{
    char **urls = calloc(1, sizeof(char*));

    urls = image_source_enumerate_v4l2(urls);
    urls = image_source_enumerate_dc1394(urls);

    return urls;
}

void image_source_enumerate_free(char **urls)
{
    if (urls == NULL)
        return;

    for (int i = 0; urls[i] != NULL; i++)
        free(urls[i]);
    free(urls);
}
