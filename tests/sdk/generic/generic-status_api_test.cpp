#include <gtest/gtest.h>
#include <aditof/status_definitions.h>
#include <aditof_test_utils.h>
#include <sstream>
#include <string>

using namespace aditof;

/**
 * API TESTS FOR STATUS AND ERROR HANDLING
 * 
 * Tests the Status enum and error handling API:
 * - Status enum values
 * - Status comparison operations
 * - Status string representation
 * - Error propagation
 */

class StatusAPITest : public ::testing::Test {
protected:
    void SetUp() override {}
    void TearDown() override {}
};

// API: Status enum - OK value
TEST_F(StatusAPITest, API_Status_OK) {
    Status status = Status::OK;
    EXPECT_EQ(status, Status::OK);
}

// API: Status enum - BUSY value
TEST_F(StatusAPITest, API_Status_BUSY) {
    Status status = Status::BUSY;
    EXPECT_EQ(status, Status::BUSY);
    EXPECT_NE(status, Status::OK);
}

// API: Status enum - UNREACHABLE value
TEST_F(StatusAPITest, API_Status_UNREACHABLE) {
    Status status = Status::UNREACHABLE;
    EXPECT_EQ(status, Status::UNREACHABLE);
    EXPECT_NE(status, Status::OK);
}

// API: Status enum - INVALID_ARGUMENT value
TEST_F(StatusAPITest, API_Status_INVALID_ARGUMENT) {
    Status status = Status::INVALID_ARGUMENT;
    EXPECT_EQ(status, Status::INVALID_ARGUMENT);
}

// API: Status enum - UNAVAILABLE value
TEST_F(StatusAPITest, API_Status_UNAVAILABLE) {
    Status status = Status::UNAVAILABLE;
    EXPECT_EQ(status, Status::UNAVAILABLE);
}

// API: Status enum - INSUFFICIENT_MEMORY value
TEST_F(StatusAPITest, API_Status_INSUFFICIENT_MEMORY) {
    Status status = Status::INSUFFICIENT_MEMORY;
    EXPECT_EQ(status, Status::INSUFFICIENT_MEMORY);
}

// API: Status enum - GENERIC_ERROR value
TEST_F(StatusAPITest, API_Status_GENERIC_ERROR) {
    Status status = Status::GENERIC_ERROR;
    EXPECT_EQ(status, Status::GENERIC_ERROR);
}

// API: Status comparison operators
TEST_F(StatusAPITest, API_Status_Comparison) {
    Status ok = Status::OK;
    Status busy = Status::BUSY;
    
    EXPECT_EQ(ok, Status::OK);
    EXPECT_NE(ok, busy);
    EXPECT_NE(busy, Status::OK);
}

// API: Status in conditional statements
TEST_F(StatusAPITest, API_Status_ConditionalsSuccess) {
    Status status = Status::OK;
    
    if (status == Status::OK) {
        EXPECT_TRUE(true);
    } else {
        FAIL() << "Conditional check failed";
    }
}

// API: Status in conditional statements - error case
TEST_F(StatusAPITest, API_Status_ConditionalsError) {
    Status status = Status::INVALID_ARGUMENT;
    
    if (status != Status::OK) {
        EXPECT_TRUE(true);
    } else {
        FAIL() << "Error condition not detected";
    }
}

// API: Status output stream operator
TEST_F(StatusAPITest, API_Status_OutputStream) {
    std::ostringstream oss;
    Status status = Status::OK;
    
    EXPECT_NO_THROW({
        oss << status;
    });
    
    std::string output = oss.str();
    EXPECT_FALSE(output.empty());
    EXPECT_NE(output.find("OK"), std::string::npos);
}

// API: Status output stream - all values
TEST_F(StatusAPITest, API_Status_OutputStreamAllValues) {
    std::vector<Status> statuses = {
        Status::OK,
        Status::BUSY,
        Status::UNREACHABLE,
        Status::INVALID_ARGUMENT,
        Status::UNAVAILABLE,
        Status::INSUFFICIENT_MEMORY,
        Status::GENERIC_ERROR
    };
    
    for (auto status : statuses) {
        std::ostringstream oss;
        EXPECT_NO_THROW({
            oss << status;
        });
        
        std::string output = oss.str();
        EXPECT_FALSE(output.empty()) << "Status output is empty";
        EXPECT_GT(output.length(), 0);
    }
}

// API: Status in boolean context
TEST_F(StatusAPITest, API_Status_BooleanContext) {
    Status ok = Status::OK;
    Status error = Status::INVALID_ARGUMENT;
    
    // Status should be comparable for success/failure
    EXPECT_TRUE(ok == Status::OK);
    EXPECT_TRUE(error != Status::OK);
}

// API: Status propagation
TEST_F(StatusAPITest, API_Status_Propagation) {
    auto func = [](Status input) -> Status {
        return input;  // Propagate status
    };
    
    Status original = Status::BUSY;
    Status propagated = func(original);
    EXPECT_EQ(original, propagated);
}

// API: Status copy semantics
TEST_F(StatusAPITest, API_Status_CopySemantics) {
    Status status1 = Status::OK;
    Status status2 = status1;
    
    EXPECT_EQ(status1, status2);
    EXPECT_EQ(status2, Status::OK);
}

// API: Status array/vector
TEST_F(StatusAPITest, API_Status_InContainer) {
    std::vector<Status> statuses;
    
    statuses.push_back(Status::OK);
    statuses.push_back(Status::BUSY);
    statuses.push_back(Status::INVALID_ARGUMENT);
    
    EXPECT_EQ(statuses.size(), 3);
    EXPECT_EQ(statuses[0], Status::OK);
    EXPECT_EQ(statuses[1], Status::BUSY);
    EXPECT_EQ(statuses[2], Status::INVALID_ARGUMENT);
}

// API: Status in map/dictionary
TEST_F(StatusAPITest, API_Status_InMap) {
    std::map<int, Status> statusMap;
    
    statusMap[0] = Status::OK;
    statusMap[1] = Status::BUSY;
    statusMap[2] = Status::INVALID_ARGUMENT;
    
    EXPECT_EQ(statusMap[0], Status::OK);
    EXPECT_EQ(statusMap[1], Status::BUSY);
}

// API: Status transition simulation
TEST_F(StatusAPITest, API_Status_StateTransition) {
    Status state = Status::OK;
    
    // Simulate state transitions
    EXPECT_EQ(state, Status::OK);
    
    state = Status::BUSY;
    EXPECT_EQ(state, Status::BUSY);
    
    state = Status::OK;
    EXPECT_EQ(state, Status::OK);
}

// API: Status error checking pattern
TEST_F(StatusAPITest, API_Status_ErrorCheckingPattern) {
    auto operation = [](int input) -> Status {
        if (input < 0) {
            return Status::INVALID_ARGUMENT;
        }
        return Status::OK;
    };
    
    Status result1 = operation(5);
    EXPECT_EQ(result1, Status::OK);
    
    Status result2 = operation(-1);
    EXPECT_EQ(result2, Status::INVALID_ARGUMENT);
}

// API: Status multiple checks
TEST_F(StatusAPITest, API_Status_MultipleChecks) {
    Status status = Status::OK;
    
    if (status == Status::OK) {
        // First check
    } else if (status == Status::BUSY) {
        FAIL();
    } else if (status == Status::INVALID_ARGUMENT) {
        FAIL();
    } else {
        FAIL();
    }
    
    EXPECT_TRUE(true);
}

int main(int argc, char** argv) {
    aditof_test::TestRunner runner(argv[0]);
    
    int initResult = runner.initialize(argc, argv);
    if (initResult != -1) {
        return initResult;
    }
    
    return runner.runTests();
}
