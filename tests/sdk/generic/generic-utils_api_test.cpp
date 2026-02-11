#include <gtest/gtest.h>
#include <aditof/utils.h>
#include <aditof_test_utils.h>
#include <string>
#include <vector>
#include <fstream>
#include <filesystem>

using namespace aditof;

/**
 * API TESTS FOR UTILITY FUNCTIONS
 * 
 * Tests utility functions provided by the SDK:
 * - String manipulation (tokenization)
 * - File operations
 * - Path operations
 * - System utilities
 */

class UtilsAPITest : public ::testing::Test {
protected:
    void SetUp() override {}
    void TearDown() override {}
};

// API: splitIntoTokens - basic usage
TEST_F(UtilsAPITest, API_SplitIntoTokens_BasicUsage) {
    std::string input = "hello,world,test";
    std::vector<std::string> tokens;
    
    EXPECT_NO_THROW({
        Utils::splitIntoTokens(input, ',', tokens);
    });
    
    EXPECT_EQ(tokens.size(), 3);
    EXPECT_EQ(tokens[0], "hello");
    EXPECT_EQ(tokens[1], "world");
    EXPECT_EQ(tokens[2], "test");
}

// API: splitIntoTokens - single token
TEST_F(UtilsAPITest, API_SplitIntoTokens_SingleToken) {
    std::string input = "hello";
    std::vector<std::string> tokens;
    
    Utils::splitIntoTokens(input, ',', tokens);
    
    EXPECT_EQ(tokens.size(), 1);
    EXPECT_EQ(tokens[0], "hello");
}

// API: splitIntoTokens - empty string
TEST_F(UtilsAPITest, API_SplitIntoTokens_EmptyString) {
    std::string input = "";
    std::vector<std::string> tokens;
    
    Utils::splitIntoTokens(input, ',', tokens);
    
    // Should have at least one (empty) token
    EXPECT_GE(tokens.size(), 1);
}

// API: splitIntoTokens - consecutive delimiters
TEST_F(UtilsAPITest, API_SplitIntoTokens_ConsecutiveDelimiters) {
    std::string input = "hello,,world";
    std::vector<std::string> tokens;
    
    Utils::splitIntoTokens(input, ',', tokens);
    
    EXPECT_EQ(tokens.size(), 3);
    EXPECT_EQ(tokens[0], "hello");
    EXPECT_EQ(tokens[1], "");  // Empty token between delimiters
    EXPECT_EQ(tokens[2], "world");
}

// API: splitIntoTokens - different delimiters
TEST_F(UtilsAPITest, API_SplitIntoTokens_DifferentDelimiters) {
    std::vector<std::pair<std::string, char>> testCases = {
        {"a:b:c", ':'},
        {"a;b;c", ';'},
        {"a|b|c", '|'},
        {"a b c", ' '}
    };
    
    for (auto [input, delimiter] : testCases) {
        std::vector<std::string> tokens;
        Utils::splitIntoTokens(input, delimiter, tokens);
        
        EXPECT_EQ(tokens.size(), 3) << "Failed for delimiter: " << delimiter;
    }
}

// API: splitIntoTokens - with spaces
TEST_F(UtilsAPITest, API_SplitIntoTokens_WithSpaces) {
    std::string input = "hello world, foo bar";
    std::vector<std::string> tokens;
    
    Utils::splitIntoTokens(input, ',', tokens);
    
    EXPECT_EQ(tokens.size(), 2);
    EXPECT_EQ(tokens[0], "hello world");
    EXPECT_EQ(tokens[1], " foo bar");
}

// API: splitIntoTokens - large number of tokens
TEST_F(UtilsAPITest, API_SplitIntoTokens_ManyTokens) {
    std::string input;
    int numTokens = 100;
    
    for (int i = 0; i < numTokens; i++) {
        if (i > 0) input += ",";
        input += "token" + std::to_string(i);
    }
    
    std::vector<std::string> tokens;
    Utils::splitIntoTokens(input, ',', tokens);
    
    EXPECT_EQ(tokens.size(), numTokens);
}

// API: getExecutableFolder - basic usage
TEST_F(UtilsAPITest, API_GetExecutableFolder_Basic) {
    std::string folder;
    
    EXPECT_NO_THROW({
        folder = Utils::getExecutableFolder();
    });
    
    EXPECT_FALSE(folder.empty());
}

// API: getExecutableFolder - returns path
TEST_F(UtilsAPITest, API_GetExecutableFolder_ReturnsPath) {
    std::string folder = Utils::getExecutableFolder();
    
    // Should not be empty
    EXPECT_FALSE(folder.empty());
    
    // Should contain path separators or be "."
    bool hasPath = (folder.find('/') != std::string::npos ||
                    folder.find('\\') != std::string::npos ||
                    folder == ".");
    EXPECT_TRUE(hasPath);
}

// API: getExecutableFolder - consistent results
TEST_F(UtilsAPITest, API_GetExecutableFolder_Consistent) {
    std::string folder1 = Utils::getExecutableFolder();
    std::string folder2 = Utils::getExecutableFolder();
    
    EXPECT_EQ(folder1, folder2);
}

// API: getExecutableFolder - not empty
TEST_F(UtilsAPITest, API_GetExecutableFolder_NotEmpty) {
    std::string folder = Utils::getExecutableFolder();
    
    EXPECT_FALSE(folder.empty());
    EXPECT_GT(folder.length(), 0);
}

// API: Utils used in error handling
TEST_F(UtilsAPITest, API_Utils_InErrorHandling) {
    std::string input = "ERROR:InvalidParameter:detail1:detail2";
    std::vector<std::string> parts;
    
    Utils::splitIntoTokens(input, ':', parts);
    
    EXPECT_EQ(parts[0], "ERROR");
    EXPECT_EQ(parts[1], "InvalidParameter");
}

// API: Utils used in configuration parsing
TEST_F(UtilsAPITest, API_Utils_InConfigParsing) {
    std::string configLine = "width=512,height=512,format=DEPTH";
    std::vector<std::string> params;
    
    Utils::splitIntoTokens(configLine, ',', params);
    
    EXPECT_EQ(params.size(), 3);
    
    for (const auto& param : params) {
        EXPECT_NE(param.find('='), std::string::npos);
    }
}

// API: Utils string manipulation pattern
TEST_F(UtilsAPITest, API_Utils_StringManipulationPattern) {
    std::string path = "/home/user/project/build/bin/test";
    std::vector<std::string> parts;
    
    Utils::splitIntoTokens(path, '/', parts);
    
    // Should have multiple path components
    EXPECT_GT(parts.size(), 1);
}

// API: Utils with CSV-like data
TEST_F(UtilsAPITest, API_Utils_CSVData) {
    std::string csvLine = "field1,field2,field3,field4,field5";
    std::vector<std::string> fields;
    
    Utils::splitIntoTokens(csvLine, ',', fields);
    
    EXPECT_EQ(fields.size(), 5);
    EXPECT_EQ(fields[0], "field1");
    EXPECT_EQ(fields[4], "field5");
}

// API: Utils with quoted content
TEST_F(UtilsAPITest, API_Utils_WithQuotedContent) {
    std::string input = "\"hello,world\",test,123";
    std::vector<std::string> tokens;
    
    Utils::splitIntoTokens(input, ',', tokens);
    
    EXPECT_GE(tokens.size(), 2);
}

// API: Utils token reuse
TEST_F(UtilsAPITest, API_Utils_TokenReuse) {
    std::vector<std::string> tokens;
    
    // First use
    Utils::splitIntoTokens("a,b,c", ',', tokens);
    EXPECT_EQ(tokens.size(), 3);
    
    // Reuse same vector (should append)
    size_t firstSize = tokens.size();
    Utils::splitIntoTokens("d,e", ',', tokens);
    EXPECT_GT(tokens.size(), firstSize);
}

// API: Utils empty token handling
TEST_F(UtilsAPITest, API_Utils_EmptyTokens) {
    std::string input = "a,,c,,e";
    std::vector<std::string> tokens;
    
    Utils::splitIntoTokens(input, ',', tokens);
    
    // Should preserve empty tokens
    EXPECT_EQ(tokens.size(), 5);
    EXPECT_EQ(tokens[1], "");
    EXPECT_EQ(tokens[3], "");
}

// API: Utils numeric string splitting
TEST_F(UtilsAPITest, API_Utils_NumericStrings) {
    std::string input = "1,2,3,4,5";
    std::vector<std::string> tokens;
    
    Utils::splitIntoTokens(input, ',', tokens);
    
    EXPECT_EQ(tokens.size(), 5);
    for (int i = 0; i < 5; i++) {
        EXPECT_EQ(tokens[i], std::to_string(i + 1));
    }
}

int main(int argc, char** argv) {
    aditof_test::TestRunner runner(argv[0]);
    
    int initResult = runner.initialize(argc, argv);
    if (initResult != -1) {
        return initResult;
    }
    
    return runner.runTests();
}
