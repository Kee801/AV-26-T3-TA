// Part A: This is an extension task that requires you to decode sensor data from CAN log files.
// CAN (Controller Area Network) is a communication standard used in automotive applications (including Redback cars)
// to allow communication between sensors and controllers.
//
// Your Task: Using the signal definitions in SteeringBench.dbc, read each CAN capture in data/
// and turn it into a CSV with one row per decoded frame:
// t,u_commanded,y_measured
// eg:
// 0,15.0,0.0
// 0.005,15.0,0.0
// ...
// where t is the frame timestamp minus the first kept frame's timestamp (s), u_commanded is
// the decoded CmdAngularRate (deg/s), and y_measured is the decoded MeasuredAngle (deg).
// The above values are not real numbers; they are only there to show the expected data output format.
// Do this for all three captures:
// data/step_test.log       ->  data/step_test.csv
// data/reversal_test.log   ->  data/reversal_test.csv
// data/deadband_test.log   ->  data/deadband_test.csv
//
// The Row type, writeCsv(), and main() below are provided -- they loop the three logs, call your
// decodeLog(), and write the CSV in exactly the format above. You just need to implement decodeLog().
//
// You do not need to use any external libraries. Use the resources below to understand how to
// extract sensor data.
// Hint: Think about manual bit masking and shifting, data types required,
// what formats are used to represent values, etc.
// Resources:
// https://www.csselectronics.com/pages/can-bus-simple-intro-tutorial
// https://www.csselectronics.com/pages/can-dbc-file-database-intro
//
// Sanity check: plot your CSVs (python3 plot_data.py) and compare against the pre-plotted
// data/*.png files -- they should match.
//
// Build & run (from the TA/ folder):
//     c++ -std=c++17 Question-A.cc -o decode
//     ./decode

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>
#include <iostream>

// One output row.
struct Row {
    double t;            // seconds since the first kept frame
    double u_commanded;  // deg/s
    double y_measured;   // deg
};

// Read the candump log at `path` and return one Row per STEER_ActuatorLog frame, in order.
// Push one Row{t, u_commanded, y_measured} per kept frame.
std::vector<Row> decodeLog(const std::string& path) {
    std::vector<Row> rows;

    // TODO: your code here
    // open the log file
    std::ifstream logFile(path);

    // read file line by line
    std::string line;

    bool firstFrame = true;
    double firstTimestamp = 0;
    while (std::getline(logFile, line))
    {
        double u_commanded = 0;
        double y_measured = 0;
        // extract timestamp
        size_t timestampEnd = line.find(')');
        std::string timestamp = line.substr(1, timestampEnd - 1);

        // interface extract
        size_t interfaceStart = timestampEnd + 2;
        size_t interfaceEnd = line.find(' ', interfaceStart);
        std::string interface = line.substr(interfaceStart, interfaceEnd - interfaceStart);

        // can frame extract
        std::string canFrame = line.substr(interfaceEnd + 1);

        size_t hashPos = canFrame.find('#');
        std::string canIdStr = canFrame.substr(0, hashPos);

        // CAN ID
        std::cout << "About to parse CAN ID: [" << canIdStr << "]" << std::endl;
        int canId = std::stoul(canIdStr, nullptr, 16);

        // CAN DATA
        std::string canData = canFrame.substr(hashPos + 1);
        int byteCount = canData.length() / 2;
        int bytes[8] = {};

        for(int i = 0; i<byteCount; i++)
        {
            std::string byteStr = canData.substr(i*2, 2);

            // convert hexadecimal string to unsigned long and store in bytes array
            
            bytes[i] = std::stoul(byteStr, nullptr, 16);
        }

        // open the dbc file
        std::ifstream dbcFile("SteeringBench.dbc");
        std::string dbcLine;

        bool isMatching = false;

        while(std::getline(dbcFile, dbcLine))
        {
            // Getting the DBC message ID
            size_t BOPos = dbcLine.find("BO_ ");
            if(BOPos != std::string::npos)
            {
                size_t start_DBC_Message_ID = BOPos+4;
                size_t end_DBC_Message_ID = dbcLine.find(" ", start_DBC_Message_ID);

                std::string DBC_Message_ID_STR = dbcLine.substr(start_DBC_Message_ID, end_DBC_Message_ID-start_DBC_Message_ID);
                
                int DBC_Message_ID = std::stoul(DBC_Message_ID_STR);
                
                // Comparing DBC message ID and CAN log ID
                if(DBC_Message_ID == canId)
                {
                    isMatching = true;
                }
                else
                {
                    isMatching = false;
                }
            }

            if(isMatching)
            {

                size_t SGPos = dbcLine.find("SG_ ");
                if(SGPos != std::string::npos)
                {
                    // get signal name
                    size_t signalNamesStart = SGPos+4;
                    size_t signalNamesEnd = dbcLine.find(" ", signalNamesStart);
                    std::string signalName = dbcLine.substr(signalNamesStart, signalNamesEnd - signalNamesStart);

                    // get starting bit
                    size_t colonPos = dbcLine.find(":", signalNamesEnd);
                    size_t starting_bit_start = colonPos+1;
                    size_t starting_bit_end = dbcLine.find("|", starting_bit_start);
                    std::string startingBitStr = dbcLine.substr(starting_bit_start, starting_bit_end - starting_bit_start);
                    
                    int startingBit = std::stoul(startingBitStr);

                    // get signal length
                    size_t atPos = dbcLine.find("@", starting_bit_end);
                    std::string signalLengthStr = dbcLine.substr(starting_bit_end+1, atPos-(starting_bit_end+1));
                    
                    int signalLength = std::stoul(signalLengthStr);

                    // get endian and signedness
                    char endian = dbcLine[atPos + 1];
                    char sign = dbcLine[atPos + 2];

                    // find factor and offset
                    size_t openBracket = dbcLine.find('(', atPos);
                    size_t commaPos = dbcLine.find(',', openBracket);
                    size_t closeBracket = dbcLine.find(')', commaPos);

                    std::string factorStr = dbcLine.substr(openBracket + 1, commaPos - openBracket - 1);
                    double factor = std::stod(factorStr);

                    std::string offsetString = dbcLine.substr(commaPos + 1, closeBracket - commaPos - 1);
                    double offset = std::stod(offsetString);
                        
                    // Little Endian LSB -> MSB 
                    unsigned long long rawValue = 0;
                    if(endian == '1')
                    {
                        unsigned long long assembledValue = 0;
                        for(int i = 0; i<8; i++)
                        {
                            assembledValue |= ((unsigned long long)bytes[i]) << (8*i);
                        }

                        // Extract the signal bits
                        if(signalLength == 64)
                        {
                            rawValue = assembledValue >> startingBit;
                        }
                        else
                        {
                            unsigned long long mask = (1ULL << signalLength) - 1;
                            rawValue =(assembledValue >> startingBit) & mask;
                        }

                    }
                    // Big Endian
                    else if(endian == '0')
                    {
                        // MSB -> LSB
                        int currentBit = startingBit;

                        for(int i = 0; i < signalLength; i++)
                        {
                            int byteIndex = currentBit / 8;
                            int bitIndex = currentBit % 8;

                            unsigned long long bit = (bytes[byteIndex] >> bitIndex) & 1;
                            rawValue = (rawValue << 1) | bit;
                                
                            if(bitIndex == 0)
                            {
                                currentBit += 15;
                            }
                            else
                            {
                                currentBit--;
                            }
                        }
                    }

                    // Signed (-)
                    long long signedRawValue;
                    if(sign == '-')
                    {
                        if(signalLength == 64)
                        {
                            signedRawValue = (long long)rawValue;
                        }
                        else
                        {
                            unsigned long long signBit = 1ULL << (signalLength - 1);

                            unsigned long long valueRange = 1ULL << signalLength;

                            if(rawValue & signBit)
                            {
                                signedRawValue = (long long)(rawValue - valueRange);
                            }
                            else
                            {
                                signedRawValue = (long long)rawValue;
                            }
                        }
                    }
                    else
                    {
                        signedRawValue = (long long)rawValue;
                    }

                    // calculate physical value
                    double physicalValue = signedRawValue * factor + offset;

                    // forming row
                    if(signalName == "CmdAngularRate")
                    {
                        u_commanded = physicalValue;
                    }

                    if(signalName == "MeasuredAngle")
                    {
                        y_measured = physicalValue;
                    }

                    
                }
                
            }
        
        }
        if(isMatching)
        {
            Row row;

            double currentTimestamp = std::stod(timestamp);
            if(firstFrame)
            {
                firstTimestamp = currentTimestamp;
                firstFrame = false;
            }

            row.t = currentTimestamp - firstTimestamp;
            row.u_commanded = u_commanded;
            row.y_measured = y_measured;

            rows.push_back(row);
        }
        

    }

    return rows;
}

// Provided -- writes the rows to a CSV in the required format. Do not change.
void writeCsv(const std::string& path, const std::vector<Row>& rows) {
    std::ofstream f(path);
    f << "t,u_commanded,y_measured\n";
    for (const Row& r : rows)
        f << r.t << "," << r.u_commanded << "," << r.y_measured << "\n";
}

// Provided -- runs decodeLog() + writeCsv() for each of the three captures.
int main() {
    const char* names[] = {"step_test", "reversal_test", "deadband_test"};
    for (const char* n : names) {
        const std::string in  = std::string("data/") + n + ".log";
        const std::string out = std::string("data/") + n + ".csv";
        const std::vector<Row> rows = decodeLog(in);
        writeCsv(out, rows);
        std::printf("%-14s %6zu frames -> %s\n", n, rows.size(), out.c_str());
    }
    return 0;
}
