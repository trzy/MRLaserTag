/*===============================================================================
Copyright (c) 2020 PTC Inc. All Rights Reserved.

Vuforia is a trademark of PTC Inc., registered in the United States and other
countries.
===============================================================================*/

#include <android/log.h>
#include <stdlib.h>
#include <string>
#include "Platform.h"

namespace Platform
{
void
log(const std::string& message)
{
    __android_log_print(ANDROID_LOG_DEBUG, "DriverTemplate", "%s", message.c_str());
}
}
