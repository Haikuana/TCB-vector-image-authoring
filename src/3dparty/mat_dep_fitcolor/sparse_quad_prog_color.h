//
// MATLAB Compiler: 7.0 (R2018b)
// Date: Thu Nov  5 22:32:55 2020
// Arguments:
// "-B""macro_default""-W""cpplib:sparse_quad_prog_color""-T""link:lib""sparse_q
// uad_prog_color"
//

#ifndef __sparse_quad_prog_color_h
#define __sparse_quad_prog_color_h 1

#if defined(__cplusplus) && !defined(mclmcrrt_h) && defined(__linux__)
#  pragma implementation "mclmcrrt.h"
#endif
#include "mclmcrrt.h"
#include "mclcppclass.h"
#ifdef __cplusplus
extern "C" {
#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_sparse_quad_prog_color_C_API 
#define LIB_sparse_quad_prog_color_C_API /* No special import/export declaration */
#endif

/* GENERAL LIBRARY FUNCTIONS -- START */

extern LIB_sparse_quad_prog_color_C_API 
bool MW_CALL_CONV sparse_quad_prog_colorInitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_sparse_quad_prog_color_C_API 
bool MW_CALL_CONV sparse_quad_prog_colorInitialize(void);

extern LIB_sparse_quad_prog_color_C_API 
void MW_CALL_CONV sparse_quad_prog_colorTerminate(void);

extern LIB_sparse_quad_prog_color_C_API 
void MW_CALL_CONV sparse_quad_prog_colorPrintStackTrace(void);

/* GENERAL LIBRARY FUNCTIONS -- END */

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

extern LIB_sparse_quad_prog_color_C_API 
bool MW_CALL_CONV mlxSparse_quad_prog_color(int nlhs, mxArray *plhs[], int nrhs, mxArray 
                                            *prhs[]);

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */

#ifdef __cplusplus
}
#endif


/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__MINGW64__)

#ifdef EXPORTING_sparse_quad_prog_color
#define PUBLIC_sparse_quad_prog_color_CPP_API __declspec(dllexport)
#else
#define PUBLIC_sparse_quad_prog_color_CPP_API __declspec(dllimport)
#endif

#define LIB_sparse_quad_prog_color_CPP_API PUBLIC_sparse_quad_prog_color_CPP_API

#else

#if !defined(LIB_sparse_quad_prog_color_CPP_API)
#if defined(LIB_sparse_quad_prog_color_C_API)
#define LIB_sparse_quad_prog_color_CPP_API LIB_sparse_quad_prog_color_C_API
#else
#define LIB_sparse_quad_prog_color_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_sparse_quad_prog_color_CPP_API void MW_CALL_CONV sparse_quad_prog_color(int nargout, mwArray& x, mwArray& Ax, const mwArray& A, const mwArray& b);

/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */
#endif

#endif
