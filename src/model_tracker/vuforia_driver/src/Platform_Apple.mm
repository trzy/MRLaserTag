/*===============================================================================
Copyright (c) 2020 PTC Inc. All Rights Reserved.

Vuforia is a trademark of PTC Inc., registered in the United States and other
countries.
===============================================================================*/
#import <Foundation/Foundation.h>
#include <stdlib.h>
#include <string>
#include "Platform.h"

namespace Platform
{
void
log(const std::string& message)
{
    NSLog(@"[TemplateDriver] %s", message.c_str());
}
}
