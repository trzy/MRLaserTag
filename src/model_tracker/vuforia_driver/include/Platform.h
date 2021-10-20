/*===============================================================================
Copyright (c) 2020 PTC Inc. All Rights Reserved.

Vuforia is a trademark of PTC Inc., registered in the United States and other
countries.
===============================================================================*/
#ifndef _PLATFORM_H_
#define _PLATFORM_H_

#include <memory>
#include <string>

/// Platform specific functionality.
namespace Platform
{
/// Output a log message.
/**
 * \param message message.
 */
void log(const std::string& message);
}

#endif // _PLATFORM_H_
