#ifndef ZWEI_CONSTRUCT_GLOBAL_H
#define ZWEI_CONSTRUCT_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(ZWEI_CONSTRUCT_LIBRARY)
#  define ZWEI_CONSTRUCT_EXPORT Q_DECL_EXPORT
#else
#  define ZWEI_CONSTRUCT_EXPORT Q_DECL_IMPORT
#endif

#endif // ZWEI_CONSTRUCT_GLOBAL_H