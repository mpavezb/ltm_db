#include <ltm_db/mongo/util.h>

namespace ltm_db_mongo {
    namespace util {

        std::string vector_to_str(const std::vector<std::string> &array) {
            if (array.empty()) return "[]";
            std::vector<std::string>::const_iterator it;
            std::stringstream ss;
            ss << "[";
            for (it = array.begin(); it != array.end(); ++it) {
                ss << "'" << *it << "', ";
            }
            ss.seekp(-2, ss.cur);
            ss << "]";
            return ss.str();
        }

        std::string vector_to_str(const std::vector<int> &array) {
            if (array.empty()) return "[]";
            std::vector<int>::const_iterator it;
            std::ostringstream ss;
            ss << "[";
            for (it = array.begin(); it != array.end(); ++it) {
                ss << *it << ", ";
            }
            ss.seekp(-2, ss.cur);
            ss << "]";
            return ss.str();
        }

        std::string vector_to_str(const std::vector<uint32_t> &array) {
            if (array.empty()) return "[]";
            std::vector<uint32_t>::const_iterator it;
            std::ostringstream ss;
            ss << "[";
            for (it = array.begin(); it != array.end(); ++it) {
                ss << *it << ", ";
            }
            ss.seekp(-2, ss.cur);
            ss << "]";
            return ss.str();
        }

    }
}
