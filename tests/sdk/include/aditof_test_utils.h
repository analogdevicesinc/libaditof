#ifndef ADITOF_TEST_UTILS_H
#define ADITOF_TEST_UTILS_H

#include <gtest/gtest.h>
#include <string>
#include <vector>
#include <map>
#include <functional>

namespace aditof_test {

// Global camera IP address (automatically available to all tests)
extern std::string g_cameraipaddress;

// Generate UTC timestamp in format: YYYYMMDD_HHMMSS
std::string getUTCTimestamp();

// JSON utility functions for camera parameter testing

// Compare two JSON files
// Returns: true if files are identical, false if differences are found
// differences: Map to store differences found (key -> pair of values from file1 and file2)
bool compareJsonFiles(const std::string& jsonFile1,
                     const std::string& jsonFile2,
                     std::map<std::string, std::pair<double, double>>& differences);

// Change multiple parameters in a JSON file
// Parameters:
//   jsonFilePath: Path to the JSON file
//   sectionKey: The numbered key (e.g., "0", "1", "3")
//   subsectionKey: The subsection name (e.g., "depth-compute", "configuration-parameters")
//   parameters: Map of parameter names to their new values
// Returns: true on success, false on failure
bool changeJsonParameter(const std::string& jsonFilePath, 
                        const std::string& sectionKey,
                        const std::string& subsectionKey,
                        const std::map<std::string, double>& parameters);

// Read multiple parameters from a JSON file
// Parameters:
//   jsonFilePath: Path to the JSON file
//   sectionKey: The numbered key (e.g., "0", "1", "3")
//   subsectionKey: The subsection name (e.g., "depth-compute", "configuration-parameters")
//   parameterKeys: Vector of parameter names to read
//   values: Map to store the read parameter name-value pairs
// Returns: true on success (all parameters found), false on failure
bool readJsonParameter(const std::string& jsonFilePath,
                      const std::string& sectionKey,
                      const std::string& subsectionKey,
                      const std::vector<std::string>& parameterKeys,
                      std::map<std::string, double>& values);

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
    
    // Get executable directory path
    std::string getExecutablePath() const { return executablePath_; }
    
private:
    std::string programName_;
    std::string execName_;
    std::string timestamp_;
    std::string executablePath_;
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
