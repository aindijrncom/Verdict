#ifndef ME_MESH_PROCESS_DEFINE_H
#define ME_MESH_PROCESS_DEFINE_H

#ifdef ME_STATIC_DEFINE
#	define ME_MESHPROCESS_API
#	define ME_MESHPROCESS_CLASS
#else
#	ifdef _MSC_VER
#		ifdef ME_MESHPROCESS_EXPORTS 
#			define ME_MESHPROCESS_API __declspec(dllexport)
#			define ME_MESHPROCESS_CLASS __declspec(dllexport)
#		else
#			define ME_MESHPROCESS_API __declspec(dllimport)
#			define ME_MESHPROCESS_CLASS __declspec(dllimport)
#		endif
#   else
#		define ME_MESHPROCESS_API
#		define ME_MESHPROCESS_CLASS
#	endif
#endif


#endif