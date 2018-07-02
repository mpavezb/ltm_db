#ifndef LTM_DB_MONGO_UTIL_H
#define LTM_DB_MONGO_UTIL_H

#include <vector>
#include <string>
#include <sstream>
#include <stdint.h>

namespace ltm_db_mongo {
    namespace util {
        std::string vector_to_str(const std::vector<std::string> &array);

        std::string vector_to_str(const std::vector<int> &array);

        std::string vector_to_str(const std::vector<uint32_t> &array);
    }
}

#endif //LTM_DB_MONGO_UTIL_H
