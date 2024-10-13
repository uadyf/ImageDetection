#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>
#include <filesystem>
#include <queue>
#include <random>
#include <chrono>
#include <fstream>
#include <conio.h>
#include <ctime>
#include <iomanip>
#include <functional>

using namespace cv;
using namespace std;

struct mPoint {
    int x, y;
public:
    mPoint(int x, int y) : x(x), y(y) {}

    friend bool operator!=(mPoint& a, mPoint& b) {
        return (a.x != b.x) || (a.y != b.y);
    }
    friend bool operator==(mPoint& a, mPoint& b) {
        return (a.x == b.x) && (a.y == b.y);
    }
    mPoint operator+(const mPoint& b) const {
        return mPoint(x + b.x, y + b.y);
    }
};

struct AStarNode {
    mPoint point;
    int gCost, hCost;
    int fCost() const {
        return gCost + hCost;
    }
    AStarNode(mPoint p, int g, int h) : point(p), gCost(g), hCost(h) {}

    bool operator>(const AStarNode& other) const {
        return fCost() > other.fCost();
    }
};

void clearScreen() {
#ifdef _WIN32
    system("cls");
#else
    cout << "\033[2J\033[H";
#endif
}

mt19937 rng(chrono::steady_clock::now().time_since_epoch().count());

vector<mPoint> direct = { mPoint(1, 0), mPoint(-1, 0), mPoint(0, -1), mPoint(0, 1) };
vector<mPoint> direct_generate = { mPoint(2, 0), mPoint(-2, 0), mPoint(0, -2), mPoint(0, 2) };
//                              D           U               L               R
string image_directory = "D:/ImageDetection/maze/scale/";
string image_directory_saved = "D:/ImageDetection/maze/saved/";
Mat image, binaryImage;
int width = 1, R, C;

void _sort(vector<string>& vc) {
    int i, j, mdx;
    for (int i = 0; i < (int)vc.size(); i++) {
        mdx = i;
        for (int j = i + 1; j < (int)vc.size(); j++) {
            if (vc[j] < vc[mdx]) {
                mdx = j;
            }
        }
        if (mdx != i) {
            swap(vc[mdx], vc[i]);
        }
    }
}

void resetPath() {
    while (image_directory.back() != '/') {
        image_directory.pop_back();
    }
}

void showPoint(mPoint& p) {
    cout << p.x << ' ' << p.y << endl;
}

void getPixelInfo() {
    for(int i = 0; i < binaryImage.rows; ++i) {
        for(int j = 0; j < binaryImage.cols; ++j) {
            cout << (binaryImage.at<uchar>(i, j) == 255 ? 1 : 0);
        }
        cout << endl;
    }
}

string chooseFile() {
    char ch;
    cout << "Continue? (y/n) ";
    cin >> ch;
    if (ch == 'n') return "";

    vector<string> all_files;
    for (const auto& files : filesystem::directory_iterator(image_directory)) {
        all_files.push_back(files.path().filename().string());
    }
    _sort(all_files);
    for (int i = 0; i < (int)all_files.size(); i++) {
        cout << i + 1 << ". " << all_files[i] << endl;
    }
    cout << all_files.size() << ' ' << "exit" << endl;

    cout << "File yang dipilih: ";
    int choose;
    cin >> choose;
    if (0 <= choose && choose <= (int)all_files.size() - 1) {
        return all_files[choose];
    }
    else {
        return "";
    }
    cout << endl;
}

int getRand(int L, int R) {
    return uniform_int_distribution<int>(L, R)(rng);
}

bool validRange(int L, int R, int pt) {
    return (L <= pt && pt <= R);
}

int dist(mPoint& a, mPoint& b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
}

vector<mPoint> BFS(vector<vector<int>>& mat, mPoint& start, mPoint& end) {
    queue<mPoint> q;
    vector<vector<bool>> vis(mat.size(), vector<bool>(mat[0].size(), 0));
    vector<vector<mPoint>> parent(mat.size(), vector<mPoint>(mat[0].size(), mPoint(-1, -1)));
    vector<mPoint> path;

    q.push(start);

    while (!q.empty()) {
        mPoint u = q.front();
        vis[u.x][u.y] = 1;
        q.pop();

        if (u == end) {
            for (mPoint i = u; i.x != -1 && i.y != -1; i = parent[i.x][i.y]) {
                path.push_back(i);
            }
            reverse(path.begin(), path.end());
            break;
        }

        for (const auto& direction : direct) {
            mPoint v(u.x + direction.x, u.y + direction.y);
            if (validRange(0, (int)mat.size() - 1, v.x) && validRange(0, (int)mat[0].size() - 1, v.y) && !vis[v.x][v.y] && mat[v.x][v.y] == 1) {
                q.push(v);
                parent[v.x][v.y] = mPoint(u.x, u.y);
            }
        }
    }
    return path;
}

vector<mPoint> AStar(vector<vector<int>>& mat, mPoint& start, mPoint& end) {
    priority_queue<AStarNode, vector<AStarNode>, greater<AStarNode>> openSet;
    vector<vector<bool>> closedSet(mat.size(), vector<bool>(mat[0].size(), false));
    vector<vector<mPoint>> parent(mat.size(), vector<mPoint>(mat[0].size(), mPoint(-1, -1)));

    openSet.emplace(start, 0, dist(start, end));

    while (!openSet.empty()) {
        AStarNode current = openSet.top();
        openSet.pop();
        mPoint u = current.point;

        if (u == end) {
            vector<mPoint> path;
            for (mPoint i = u; i.x != -1 && i.y != -1; i = parent[i.x][i.y]) {
                path.push_back(i);
            }
            reverse(path.begin(), path.end());
            return path;
        }

        closedSet[u.x][u.y] = true;

        for (const auto& direction : direct) {
            mPoint v(u.x + direction.x, u.y + direction.y);
            if (v.x >= 0 && v.x < (int)mat.size() && v.y >= 0 && v.y < (int)mat[0].size() && mat[v.x][v.y] == 1 && !closedSet[v.x][v.y]) {
                int newGCost = current.gCost + 1;
                int newHCost = dist(v, end);
                if (newGCost < current.gCost || !closedSet[v.x][v.y]) {
                    openSet.emplace(v, newGCost, newHCost);
                    parent[v.x][v.y] = u;
                }
            }
        }
    }
    return {};  // Return empty path if no solution found
}
//
void saveFile(Mat& mat, Mat& mat_noscale, string& name) {
    // save file with scale
    auto now = chrono::system_clock::now();
    time_t now_time = chrono::system_clock::to_time_t(now);
    tm local_time;

    if (localtime_s(&local_time, &now_time) != 0) {
        cerr << "Failed to get local time" << endl;
    }

    ostringstream oss;
    oss << put_time(&local_time, "%Y-%m-%d");
    string time_string = oss.str();
    //cout << time_string << endl;
    string file_name = image_directory_saved + name + "_" + time_string + ".png";

    if (!imwrite(file_name, mat)) {
        cerr << "Failed to save the image to " << file_name << endl;
        return;
    }
    else {
        cout << endl;
        cout << "Your file has been saved. " << file_name << endl;
    }

    ofstream maze_history(image_directory_saved + "Maze History.txt", ios::app);
    maze_history << file_name << endl;
    maze_history.close();

    // save file no scale (scale = 1)
    string dir_noscale = image_directory + name + "_" + time_string + ".png";
    if (!imwrite(dir_noscale, mat_noscale)) {
        cerr << "Failed to save no scale image to" << dir_noscale << endl;
    }
}

void drawMaze(Mat& blank_image, vector<vector<int>>& mat, int scale, bool show_image = true, int hold = 0) {
    for (int i = 0, y_image = 0; i < mat.size(); i++, y_image += scale) {
        for (int j = 0, x_image = 0; j < mat[0].size(); j++, x_image += scale) {
            if (mat[i][j] == 1) {
                Point top_left(x_image, y_image);
                Point bot_right(x_image + scale - 1, y_image + scale - 1);
                rectangle(blank_image, top_left, bot_right, Scalar(255, 255, 255), FILLED);
            }
        }
    }
    if (show_image) {
        imshow("Maze", blank_image);
        waitKey(hold);
    }
}

void randomDFS(vector<vector<int>>& mat, mPoint curr) {
    mat[curr.x][curr.y] = 1;
    shuffle(direct_generate.begin(), direct_generate.end(), rng);

    for (const mPoint& direction : direct_generate) {
        mPoint nxt = curr + direction;
        if (validRange(1, R - 2, nxt.x) && validRange(1, C - 2, nxt.y) && mat[nxt.x][nxt.y] == 0) {
            mat[curr.x + direction.x / 2][curr.y + direction.y / 2] = 1;
            randomDFS(mat, nxt);
        }
    }
}

mPoint generate_start(int entrance_input) {
    mPoint entrance(-1, -1);
    if (entrance_input == 1) { // UP
        entrance = mPoint(0, getRand(1, C - 2));
    }
    else if (entrance_input == 2) { // DOWN
        entrance = mPoint(R - 1, getRand(1, C - 2));
    }
    else if (entrance_input == 3) { // LEFT
        entrance = mPoint(getRand(1, R - 2), 0);
    }
    else { // RIGHT
        entrance = mPoint(getRand(1, R - 2), C - 1);
    }
    return entrance;
}

void generateMaze() {
    char ch;
    cout << "Continue? (y/n) ";
    cin >> ch;
    if (ch == 'n') return;
    cout << endl;
    cout << "Rows\t: ";
    cin >> R;
    cout << "Columns\t: ";
    cin >> C;
    cout << endl;
    vector<vector<int>> raw_mat(R, vector<int>(C, 0));

    int entrance_input;
    cout << "Entrance?" << endl;
    cout << "1. up" << endl;
    cout << "2. down" << endl;
    cout << "3. left" << endl;
    cout << "4. right" << endl;
    cout << "5. i'm feeling lucky" << endl;
    cin >> entrance_input;
    cout << endl;

    if (entrance_input == 5) {
        entrance_input = getRand(1, 4);
    }

    for (int i = 0; i < R; i++) {
        raw_mat[i][0] = raw_mat[i][C - 1] = 0;
    }
    for (int j = 0; j < C; j++) {
        raw_mat[0][j] = raw_mat[R - 1][j] = 0;
    }

    mPoint entrance(-1, -1), end(-1, -1);

    // get input point
    entrance = generate_start(entrance_input);
    randomDFS(raw_mat, entrance);
    
    // get end point & connect
    end = generate_start(entrance_input + (entrance_input % 2 ? 1 : -1));
    mPoint closest(-1, -1);
    int closest_dist = 1e9;
    for (int i = 0; i < R; i++) {
        for (int j = 0; j < C; j++) {
            mPoint temp = mPoint(i, j);
            if (temp != end && raw_mat[i][j] == 1 && closest_dist > dist(end, temp)) {
                closest_dist = dist(end, temp);
                closest = temp;
            }
        }
    }
    vector<mPoint> connect_end = BFS(raw_mat, end, closest);
    //showPoint(closest);
    //for (auto [i, j] : connect_end) {
    //    raw_mat[i][j] = 1;
    //    cout << i << ' ' << j << endl;
    //}
    raw_mat[end.x][end.y] = 1;

    // DEBUG
    //for (int i = 0; i < (int)raw_mat.size(); i++) {
    //    for (int j = 0; j < (int)raw_mat[0].size(); j++) {
    //        cout << raw_mat[i][j];
    //    }
    //    cout << endl;
    //}
    //

    int scale = 10;
    while (scale >= 0 && R * scale > 700 && C * scale > 800) scale--;
    scale = max(scale, 1);
    Mat generated_maze(R * scale, C * scale, CV_8UC3, Scalar(0, 0, 0));
    Mat noscale_generated(R, C, CV_8UC3, Scalar(0, 0, 0));
    drawMaze(generated_maze, raw_mat, scale);
    drawMaze(noscale_generated, raw_mat, 1, false);

    char choose;
    cout << "Do you want to save your file? (y/n) ";
    cin >> choose;
    if (choose == 'y') {
        string name;
        cout << "Name of your file : ";
        cin >> name;
        saveFile(generated_maze, noscale_generated, name);
    }
}
//
vector<vector<int>> simplify() {
    vector<vector<int>> result;
    for (int i_vector = 0, i = 0; i < binaryImage.rows; i += width, i_vector++) {
        result.push_back({});
        for (int j = 0; j < binaryImage.cols; j += width) {
            if (binaryImage.at<uchar>(i, j) == 255) {
                result[i_vector].push_back(1);
            }
            else {
                result[i_vector].push_back(0);
            }
        }
    }
    return result;
}

mPoint findFirstOne(vector<vector<int>> &mat, int pivot_row, int pivot_col) {
    mPoint res(-1, -1);
    for (int j = 0; j < (int)mat[0].size() && res.x == -1 && res.y == -1; j++) {
        if (mat[pivot_row][j] == 1) {
            res = mPoint(pivot_row, j);
        }
    }
    for (int i = 0; i < (int)mat.size() && res.x == -1 && res.y == -1; i++) {
        if (mat[i][pivot_col] == 1) {
            res = mPoint(i, pivot_col);
        }
    }
    return res;
}

void drawPath(Mat& blank_image, vector<vector<int>>& mat, vector<mPoint>& path, int scale) {
    for (int i = 0, y_image = 0; i < mat.size(); i++, y_image += scale) {
        for (int j = 0, x_image = 0; j < mat[0].size(); j++, x_image += scale) {
            if (mat[i][j] == 1) {
                Point top_left(x_image, y_image);
                Point bot_right(x_image + scale - 1, y_image + scale - 1);
                rectangle(blank_image, top_left, bot_right, Scalar(255, 255, 255), FILLED);
            }
        }
    }
    imshow("Maze", blank_image);
    waitKey(10);
    for (const mPoint& it : path) {
        Point top_left(it.y * scale, it.x * scale);
        Point bot_right(it.y * scale + scale - 1, it.x * scale + scale - 1);
        rectangle(blank_image, top_left, bot_right, Scalar(0, 0, 255), FILLED);
        imshow("Maze", blank_image);
        waitKey(3);
    }
    waitKey(0);
}

void solveMaze() {
    string res = chooseFile();
    if (res == "") {
        return;
    }
    image_directory += res;
    image = imread(image_directory, IMREAD_GRAYSCALE);

    threshold(image, binaryImage, 128, 255, THRESH_BINARY);

    vector<vector<int>> reduct_mat = simplify();

    mPoint start = findFirstOne(reduct_mat, 0, 0);
    mPoint end = findFirstOne(reduct_mat, (int)reduct_mat.size() - 1, (int)reduct_mat[0].size() - 1);

    //for (int i = 0; i < reduct_mat.size(); i++) {
    //    for (int j = 0; j < reduct_mat[0].size(); j++) {
    //        cout << reduct_mat[i][j];
    //    }
    //    cout << endl;
    //}

    int ch;
    vector<mPoint> solve_result;
    cout << endl;
    cout << "Choose your algorithm:" << endl;
    cout << "1. Breadth First Search" << endl;
    cout << "2. A* Algorithm" << endl;
    cout << "Choice: ";
    cin >> ch;
    auto start_time = chrono::steady_clock::now();
    if (ch == 1) {
        solve_result = BFS(reduct_mat, start, end);
    }
    else {
        solve_result = AStar(reduct_mat, start, end);
    }
    auto end_time = chrono::steady_clock::now();
    auto diff = end_time - start_time;

    int scale = 10;
    for (; scale >= 0; scale--) {
        if (reduct_mat.size() * scale < 700 && reduct_mat[0].size() * scale < 800) {
            break;
        }
    }
    Mat result_image(reduct_mat.size() * scale, (int)reduct_mat[0].size() * scale, CV_8UC3, Scalar(0, 0, 0));
    drawPath(result_image, reduct_mat, solve_result, scale);
    auto duration = chrono::duration_cast<chrono::microseconds>(diff);
    cout << endl;
    cout << "Time to run the program : " << duration.count() << " ms" << endl;
    resetPath();
}

void displayMenu(const vector<string>& options, vector<string>::size_type selectedIndex) {
    clearScreen();
    for (vector<string>::size_type i = 0; i < options.size(); ++i) {
        if (i == selectedIndex) {
            cout << "> " << options[i] << endl;
        }
        else {
            cout << "  " << options[i] << endl;
        }
    }
}

void showMenu() {
    cout << "\tMaze Solver & Generator" << endl;

    vector<string> options = {
        "Generate Puzzle",
        "Solve Puzzle",
        "Exit Program"
    };
    vector<string>::size_type selectedIndex = 0;

    displayMenu(options, selectedIndex);

    while (true) {
        if (_kbhit()) {
            int key = _getch();

            // Arrow keys are two-part sequences: 224 followed by another value.
            if (key == 224) {
                key = _getch();

                if (key == 72) { // Up arrow
                    if (selectedIndex > 0) {
                        --selectedIndex;
                    }
                }
                else if (key == 80) { // Down arrow
                    if (selectedIndex < options.size() - 1) {
                        ++selectedIndex;
                    }
                }
                displayMenu(options, selectedIndex);
            }
            else if (key == 13) { // Enter key
                clearScreen();
                if (selectedIndex == 0) {
                    generateMaze();
                }
                else if (selectedIndex == 1) {
                    solveMaze();
                }
                else if (selectedIndex == 2) {
                    cout << "Program is closing..." << endl;
                    break;
                }
                cout << "Press any key to continue..." << endl;
                _getch(); // Wait for a key press
                displayMenu(options, selectedIndex);
            }
        }
        destroyAllWindows();
    }
}

int main() {
    showMenu();
}