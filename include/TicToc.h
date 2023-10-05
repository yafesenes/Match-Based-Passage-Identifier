#ifndef TICTOC_CLASS_H
#define TICTOC_CLASS_H
#include <chrono>
#include <iostream>
#include <iomanip>
#include <unordered_map>
#include <vector>
#include <assert.h>

using namespace std;
using namespace std::chrono;

inline unordered_map<string, pair<high_resolution_clock::time_point, int>> startsMap;
inline unordered_map<string, size_t> durationMap;
inline unordered_map<string, int> callCountMap;

inline void tic(const string& MissionName)
{
    startsMap[MissionName].second++;

    if(callCountMap.find(MissionName) == callCountMap.end())
        callCountMap[MissionName] = 0;

    callCountMap[MissionName] += 1;
    if (callCountMap[MissionName] == 1)
    {
        startsMap[MissionName].first = high_resolution_clock::now();
    }
}
    

inline void toc(const string& MissionName)
{
    if (callCountMap[MissionName] != 1)
    {
        callCountMap[MissionName] -= 1;
        return;
    }
    
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - startsMap[MissionName].first);

    if (durationMap.find(MissionName) == durationMap.end())
        durationMap[MissionName] = 0;

    durationMap[MissionName] += duration.count();
    callCountMap[MissionName] -= 1;
}

inline bool value_comparator(const pair<string, size_t>& a, const pair<string, size_t>& b) {
    return a.second > b.second;
}

inline void printAllTimes()
{
    vector<pair<string, size_t>> durations(durationMap.begin(), durationMap.end());
    cout << endl << "Profiler: " << endl;

    for (auto& i : startsMap)
    {
        if (callCountMap[i.first] != 0)
        {
            cerr << i.first << " tic-tocs does not match!" << endl;
            return;
        }
    }

    sort(durations.begin(), durations.end(), value_comparator);

    int num = 1;
    for (const auto& i : durations)
    {
        cout << setw(3) << num << ". ";
        cout << setw(30) << left << i.first;
        cout << "-> " << setw(7) << right << i.second / 1000;
        cout << " milliseconds,    -> " << setw(10) << startsMap[i.first].second << " times called" << endl;
        num++;
    }
}

 
#endif