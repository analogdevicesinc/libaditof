/**********************************************************************************/
/* MIT License                                                                    */
/*                                                                                */
/* Copyright (c) 2021 Analog Devices, Inc.                                        */
/*                                                                                */
/* Permission is hereby granted, free of charge, to any person obtaining a copy   */
/* of this software and associated documentation files (the "Software"), to deal  */
/* in the Software without restriction, including without limitation the rights   */
/* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell      */
/* copies of the Software, and to permit persons to whom the Software is          */
/* furnished to do so, subject to the following conditions:                       */
/*                                                                                */
/* The above copyright notice and this permission notice shall be included in all */
/* copies or substantial portions of the Software.                                */
/*                                                                                */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR     */
/* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,       */
/* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE    */
/* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER         */
/* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,  */
/* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE  */
/* SOFTWARE.                                                                      */
/**********************************************************************************/

#include "command_parser.h"

void CommandParser::addCommand(
    const std::string &command, const std::string &value,
    std::vector<std::pair<std::string, std::string>> &m_command_vector) {
    m_command_vector.push_back({command, value});
}

void CommandParser::getMandatoryArgs(
    const int &argc, const std::map<std::string, struct Argument> &command_map,
    std::vector<std::pair<std::string, int>> &arg_position) {
    for (auto ct = command_map.begin(); ct != command_map.end(); ct++) {
        if (ct->second.is_mandatory == true) {
            if (ct->second.position == "last") {
                arg_position.push_back({ct->first, argc - 1});
            } else {
                arg_position.push_back(
                    {ct->first, std::stoi(ct->second.position)});
            }
        }
    }
}

bool CommandParser::isHelpArg(const std::string &arg) {
    return arg.find("-h") != std::string::npos ||
           arg.find("--help") != std::string::npos;
}

bool CommandParser::hasEqual(const std::string &arg) {
    return arg.find("=") != std::string::npos;
}

bool CommandParser::nextArgIsCommand(const std::string &arg) {
    return (arg.substr(0, 2) == "--") || (arg.substr(0, 1) == "-");
    //return arg.find("-") != std::string::npos;
}

void CommandParser::processFlag(
    const std::string &arg, int &arg_number,
    std::vector<std::pair<std::string, std::string>> &m_command_vector) {
    addCommand(arg, "true", m_command_vector);
    arg_number++;
}

void CommandParser::processEqualArg(
    const std::string &arg, int &arg_number, const int &euqal_pos,
    std::vector<std::pair<std::string, std::string>> &m_command_vector) {
    addCommand(arg.substr(0, euqal_pos), arg.substr(euqal_pos + 1),
               m_command_vector);
    arg_number++;
}

void CommandParser::processNonEqualArg(
    const std::string &arg, const std::string &value, int &arg_number,
    std::vector<std::pair<std::string, std::string>> &m_command_vector) {
    addCommand(arg, value, m_command_vector);
    arg_number += 2;
}

void CommandParser::processLongArg(
    const std::string &arg, int &arg_number,
    std::vector<std::pair<std::string, std::string>> &m_command_vector,
    const std::map<std::string, struct Argument> &command_map) {
    bool arg_found = false;
    for (const auto &entry : command_map) {
        const Argument &argument = entry.second;
        if (argument.long_option == arg) {
            arg_found = true;
            if (argument.has_value) {
                addCommand(arg, "", m_command_vector);
                arg_number++;
            } else {
                processFlag(arg, arg_number, m_command_vector);
            }
            break;
        }
    }
    if (!arg_found) {
        addCommand(arg, "", m_command_vector);
        arg_number++;
    }
}

void CommandParser::processShorArg(
    const std::string &arg, int &arg_number,
    std::vector<std::pair<std::string, std::string>> &m_command_vector,
    const std::map<std::string, struct Argument> &command_map) {
    auto command = command_map.find(arg);
    if (command != command_map.end()) {
        if (command->second.has_value) {
            addCommand(arg, "", m_command_vector);
            arg_number++;
        } else {
            processFlag(arg, arg_number, m_command_vector);
        }
    } else {
        addCommand(arg, "", m_command_vector);
        arg_number++;
    }
}

void CommandParser::parseArguments(
    int argc, char *argv[],
    std::map<std::string, struct Argument> command_map) {
    int arg_number = 1;

    // Stores mandatory arguments and their location
    std::vector<std::pair<std::string, int>> arg_position;

    getMandatoryArgs(argc, command_map, arg_position);

    for (int i = 1; i < argc; i++) {
        bool mandatory = false;
        int equal_pos = std::string(argv[i]).find("=");
        for (int j = 0; j < arg_position.size(); j++) {
            if (arg_number == arg_position[j].second && !isHelpArg(argv[i])) {
                if (hasEqual(argv[i])) {
                    processEqualArg(argv[i], arg_number, equal_pos,
                                    m_command_vector);
                } else if (arg_number != argc - 1) {
                    processNonEqualArg(argv[i], argv[i + 1], arg_number,
                                       m_command_vector);
                    i++;
                } else {
                    addCommand(arg_position[j].first, argv[i],
                               m_command_vector);
                }
                mandatory = true;
                break;
            }
        }
        if (mandatory) {
            continue;
        }
        if (isHelpArg(argv[i])) {
            processFlag(argv[i], arg_number, m_command_vector);
            continue;
        }
        if (hasEqual(argv[i])) { // Solves -arg/--arg=value
            processEqualArg(argv[i], arg_number, equal_pos, m_command_vector);
            continue;
        }
        if (i < argc - 1 && !hasEqual(argv[i]) &&
            !nextArgIsCommand(argv[i + 1])) { // Solves -arg/--arg value
            processNonEqualArg(argv[i], argv[i + 1], arg_number,
                               m_command_vector);
            i++;
            continue;
        }
        if (std::string(argv[i]).find("--") !=
            -1) { // Solves -arg/--arg -arg value
            processLongArg(argv[i], arg_number, m_command_vector, command_map);
        } else {
            processShorArg(argv[i], arg_number, m_command_vector, command_map);
        }
    }
}

int CommandParser::checkArgumentExist(
    std::map<std::string, struct Argument> &command_map,
    std::string &arg_error) {
    // Check if arguments used exist
    for (int i = 0; i < m_command_vector.size(); i++) {
        bool is_command = false;
        for (auto ct = command_map.begin(); ct != command_map.end(); ct++) {
            if (m_command_vector[i].first == ct->first ||
                m_command_vector[i].first == ct->second.long_option) {
                is_command = true;
                break;
            }
        }
        if (!is_command) {
            arg_error = m_command_vector[i].first;
            return -1;
        }
    }
    return 0;
}

int CommandParser::helpMenu() {
    for (int i = 0; i < m_command_vector.size(); i++) {
        if (m_command_vector[i].first == "-h" ||
            m_command_vector[i].first == "--help") {
            if (i != 0 || m_command_vector.size() != 1) {
                return -1;
            }
            return 1;
        }
    }
    return 0;
}

int CommandParser::checkValue(
    std::map<std::string, struct Argument> &command_map,
    std::string &arg_error) {
    // Checks if argument has default and value assigned.
    // If there is value assigned, it will send it
    for (int i = 0; i < m_command_vector.size(); i++) {
        for (auto ct = command_map.begin(); ct != command_map.end(); ct++) {
            if (m_command_vector[i].first == ct->first ||
                m_command_vector[i].first == ct->second.long_option) {
                if (m_command_vector[i].second == "" &&
                    ct->second.value == "") {
                    arg_error = ct->first;
                    return -1;
                } else if (m_command_vector[i].second == "" &&
                           ct->second.value != "") {
                    // Argument doesn't have value assigned but has default
                    break;
                } else {
                    // Argument has value assigned
                    ct->second.value = m_command_vector[i].second;
                    break;
                }
            }
        }
    }
    return 0;
}

int CommandParser::checkMandatoryArguments(
    std::map<std::string, struct Argument> &command_map,
    std::string &arg_error) {
    // Check if mandatory arguments are provided
    for (auto ct = command_map.begin(); ct != command_map.end(); ct++) {
        if (ct->second.is_mandatory == true && ct->second.value == "") {
            arg_error = ct->first;
            return -1;
        }
    }
    return 0;
}

int CommandParser::checkMandatoryPosition(
    std::map<std::string, struct Argument> &command_map,
    std::string &arg_error) {
    // Mandatory arguments location check
    for (auto ct = command_map.begin(); ct != command_map.end(); ct++) {
        if (ct->second.is_mandatory == true) {
            int index;
            if (ct->second.position != "last") {
                index = std::stoi(ct->second.position) - 1;
            } else {
                index = m_command_vector.size() - 1;
            }
            if (m_command_vector[index].first != ct->first &&
                m_command_vector[index].first != ct->second.long_option) {
                arg_error = ct->first;
                return -1;
            }
        }
    }
    return 0;
}
