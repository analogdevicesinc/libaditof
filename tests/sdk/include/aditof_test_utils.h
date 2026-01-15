#ifndef ADITOF_TEST_UTILS_H
#define ADITOF_TEST_UTILS_H

#include <gtest/gtest.h>
#include <string>
#include <vector>
#include <map>
#include <functional>

namespace aditof_test {

// Generate UTC timestamp in format: YYYYMMDD_HHMMSS
std::string getUTCTimestamp();

// Structure to define custom command-line arguments
struct CustomArg {
    enum Type { STRING, UINT16, BOOL };
    
    std::string prefix;           // e.g., "--ip="
    Type type;
    std::string* targetString;    // Pointer to string to store value
    uint16_t* targetUint16;       // Pointer to uint16_t to store value
    bool* targetBool;             // Pointer to bool flag
    std::string description;      // Help text
    
    CustomArg(const std::string& p, std::string* t, const std::string& d)
        : prefix(p), type(STRING), targetString(t), targetUint16(nullptr), targetBool(nullptr), description(d) {}
    
    CustomArg(const std::string& p, uint16_t* t, const std::string& d)
        : prefix(p), type(UINT16), targetString(nullptr), targetUint16(t), targetBool(nullptr), description(d) {}
    
    CustomArg(const std::string& p, bool* t, const std::string& d)
        : prefix(p), type(BOOL), targetString(nullptr), targetUint16(nullptr), targetBool(t), description(d) {}
};

// Main test runner configuration
class TestRunner {
public:
    TestRunner(const std::string& programName);
    
    // Add custom arguments
    void addArgument(const CustomArg& arg);
    
    // Parse arguments and initialize GTest
    // Returns: -1 if should continue, 0 if help shown, 1 if error
    int initialize(int& argc, char** argv);
    
    // Set validation callback to run before tests
    void setPreTestValidator(std::function<bool()> validator);
    
    // Enable/disable strict argument checking (default: true)
    void setStrictArguments(bool strict);
    
    // Run all tests
    int runTests();
    
    // Get timestamp
    std::string getTimestamp() const { return timestamp_; }
    
private:
    std::string programName_;
    std::string execName_;
    std::string timestamp_;
    std::vector<CustomArg> customArgs_;
    std::vector<std::pair<std::string, std::string>> propertiesToRecord_;
    bool helpRequested_;
    bool strictArgs_;
    std::function<bool()> preTestValidator_;
    
    // Modified argv for GTest
    std::vector<char*> newArgv_;
    std::string gtestOutput_;
    int newArgc_;
};

} // namespace aditof_test

#endif // ADITOF_TEST_UTILS_H
