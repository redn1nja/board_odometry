#include "Common.h"

#include <easymedia/rkmedia_api.h>

#include <iostream>

namespace vision {

void InitRkmedia()
{
    int ret = RK_MPI_SYS_Init();
    if (ret) {
        std::cout << "Failed to initialize RK_MPI_SYS" << std::endl;
    }
}

}