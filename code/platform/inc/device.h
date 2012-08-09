#ifndef __DEVICE_H__
#define __DEVICE_H__
#include "type.h"
#include "log.h"

struct device {
    struct device_driver *driver;    /* which driver has allocated this device */
    void                 *platform_data;    /* Platform specific data, device core doesn't touch it */
};

struct device_driver {
    const char        *name;
    int  (*probe) (struct device *dev);
    int  (*remove) (struct device *dev);
    void (*shutdown) (struct device *dev);
    int  (*suspend) (struct device *dev);
    int  (*resume) (struct device *dev);

};

struct platform_device {
    const char    * name;
    int             id;
    struct device   dev;
};

#endif
