//
// Created by LiuQiang on 2024/9/29.
//

// crt_dupenv_s.c
#include  <stdlib.h>
#include "Logging.h"
#include <crtdbg.h>

int main( void )
{
    char *environment_terminal;
    size_t len;
    errno_t err = _dupenv_s_dbg(&environment_terminal, &len, "GNUPLOT",  _NORMAL_BLOCK, __FILE__, __LINE__);
    const bool env_found = err == 0 && environment_terminal != nullptr;

    char *pValue;
    err = _dupenv_s( &pValue, &len, "pathext" );
    if ( err ) return -1;
    printf( "pathext = %s\n", pValue );
    free( pValue );
    err = _dupenv_s( &pValue, &len, "nonexistentvariable" );
    if ( err ) return -1;
    printf( "nonexistentvariable = %s\n", pValue );
    free( pValue ); // It's OK to call free with NULL
}