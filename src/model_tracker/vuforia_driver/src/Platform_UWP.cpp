/*===============================================================================
Copyright (c) 2020 PTC Inc. All Rights Reserved.

Vuforia is a trademark of PTC Inc., registered in the United States and other
countries.
===============================================================================*/
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <memory>
#include <string>
#include "Platform.h"

namespace Platform
{
void
log(const std::string& message)
{
    std::string messageWithTag = "[DriverTemplate] ";
    messageWithTag += message;
    messageWithTag += "\n";
    OutputDebugStringA(messageWithTag.c_str());
}
}
