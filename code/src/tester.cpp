// //
// // Created by ranai on 11/8/25.
// //
//
// #ifndef PLANNING_HW3_TESTER_H
// #define PLANNING_HW3_TESTER_H
// #include <string>
// #include <vector>
// #include <bits/stdc++.h>
//
//
//
// std::vector<std::string> getAllSymbolPermutations(const std::vector<char> &s, int k) {
//     int n = (int)s.size();
//     if (k < 0 || k > n) return {};
//
//     std::vector<std::string> result;
//     std::string cur;
//     cur.reserve(k);
//     std::vector<bool> used(n, false);
//
//     // recursive backtracking
//     std::function<void()> dfs = [&]() {
//         if ((int)cur.size() == k) {
//             result.push_back(cur);
//             return;
//         }
//         for (int i = 0; i < n; ++i) {
//             if (used[i]) continue;
//             used[i] = true;
//             cur.push_back(chars[i]);
//             dfs();
//             cur.pop_back();
//             used[i] = false;
//         }
//     };
//
//     dfs();
//     return result;
// }
//
// int main() {
//     std::vector<std::string> retval = getAllSymbolPermutations("ABCD", 3);
//
//     for (std::string s: retval) {
//         std::cout << s << std::endl;
//     }
//
// }
//
//
// #endif //PLANNING_HW3_TESTER_H