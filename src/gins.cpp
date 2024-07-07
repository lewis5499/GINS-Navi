#include <iostream>
#include <Eigen/Geometry>
#include <matplotlibcpp.h>
#include <thread_pool.hpp>
#include <thread_pool_utils.hpp>
#include "plotter.h"
#include "rtkprocess.h"
#include "fileloader.h"
#include "filesaver.h"
#include "imutypes.h"
#include "gnsstypes.h"
#include "navstate.h"
#include "timesys.h"
#include "manager.h"

using namespace std;
using namespace ThreadPool;
using namespace Manager;
using namespace timesys;
using namespace nav;
using namespace gnss;
using namespace ins;
using namespace file;
using namespace Eigen;
namespace plt = matplotlibcpp;

static FILE *GINSLogFile;
static BasicParams params;
int Run(GINSOptions &options);

#ifndef WIN32 /* Linux Console UI */

int main(int argc, char **argv) {
    GINSOptions options(_PATH_CONFIGURATION_FILE_);
    return Run(options);
}

#else /* Windows Graphics UI */

#include "resource.h"
#include <atomic>
#include <commctrl.h>
#include <cstring>
#include <direct.h>
#include <shellapi.h>
#include <strsafe.h>
#include <thread>
#include <windows.h>

#define SUBMIT_RNXObsRov 1
#define SUBMIT_RNXObsBas 2
#define SUBMIT_RNXNav 3
#define SUBMIT_RTKSolPath 4
#define SUBMIT_IMUPath 5
#define SUBMIT_OUTPUTPath 6
#define SUBMIT_ODOPath 7
#define SUBMIT_CFGPath 8
#define SUBMIT_Execute 9
#define SUBMIT_Help 10
#define SUBMIT_OpenLog 11
#define SUBMIT_pBar 12
#define SUBMIT_ClearText 13

#define WINDOW_WIDTH 710
#define WINDOW_HEIGHT 490

#define ID_TIMER 201
#define UPDATE_INTERVAL 20

HWND TextBoxTimeStart;
HWND TextBoxTimeEnd;
HWND TextBoxRNXObsRov;
HWND TextBoxRNXObsBas;
HWND TextBoxRNXNav;
HWND TextBoxRTKSolPath;
HWND TextBoxIMUPath;
HWND TextBoxOUTPUTPath;
HWND TextBoxODOPath;
HWND TextBoxCFGPath;

HWND ButtonRNXObsRov;
HWND ButtonRNXObsBas;
HWND ButtonRNXNav;
HWND ButtonRTKSolPath;
HWND ButtonIMUPath;
HWND ButtonOUTPUTPath;
HWND ButtonODOPath;
HWND ButtonCFGPath;
HWND ButtonExecute;
HWND ButtonHelp;
HWND ButtonOpenLog;
HWND ButtonClearTextBox;

HWND hProgressBar;

HFONT hFont;

static int step       = 0;
static BOOL bReverse  = FALSE;
static bool setConfig = false;
std::atomic<bool> isRunning(false);
LRESULT CALLBACK WindowProcedure(HWND, UINT, WPARAM, LPARAM);
void AddControls(HWND);
void EnableDPIAwareness();
void RunTask(HWND hWnd);
void UpdateProgressBar();
void ClearTextBox();
void ParseFileNameFromTextBox(BasicParams &fps);
void FillTextBoxesFromGINSOptions(const GINSOptions &opts);
int OpenFiletoTextBox(HWND hWnd, WPARAM wp);
int ChangeWorkDirToExe();

int WINAPI WinMain(HINSTANCE hInst, HINSTANCE hPrevInst, LPSTR atgs, int ncmdshow) {
    EnableDPIAwareness();

    ShowWindow(GetConsoleWindow(), SW_HIDE);

    WNDCLASSA wc = {0}; // 窗口结构。

    wc.hbrBackground = CreateSolidBrush(RGB(255, 255, 255));
    wc.hCursor       = LoadCursor(NULL, IDC_ARROW);                      // 加载光标。定义光标类型。
    wc.hInstance     = hInst;                                            // 操作系统用于标识窗口。
    wc.lpszClassName = "myWindowClass";                                  // 类名。
    wc.lpfnWndProc   = WindowProcedure;                                  // 指向窗口过程的指针。
    wc.hIcon         = LoadIcon(hInst, MAKEINTRESOURCE(IDI_ICON_LARGE)); // 加载大图标资源

    if (!RegisterClassA(&wc))
        return -1; // 如果有错误，返回 -1。

    // 获取屏幕的宽度和高度
    int screenWidth  = GetSystemMetrics(SM_CXSCREEN);
    int screenHeight = GetSystemMetrics(SM_CYSCREEN);

    // 计算窗口的位置以使其居中
    int posX = (screenWidth - WINDOW_WIDTH) / 2;
    int posY = (screenHeight - WINDOW_HEIGHT) / 2;

    // 创建字体对象
    hFont = CreateFontA(26, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS,
                        CLIP_DEFAULT_PRECIS, PROOF_QUALITY, DEFAULT_PITCH | FF_SWISS, "Microsoft YaHei");

    // 创建窗口
    HWND hWnd = CreateWindowA("myWindowClass", "GINS Navi",
                              WS_OVERLAPPEDWINDOW & ~WS_MAXIMIZEBOX & ~WS_MINIMIZEBOX & ~WS_THICKFRAME | WS_VISIBLE,
                              posX, posY, WINDOW_WIDTH, WINDOW_HEIGHT, NULL, NULL, hInst, NULL);

    // 设置窗口图标
    SendMessage(hWnd, WM_SETICON, ICON_BIG, (LPARAM) LoadIcon(hInst, MAKEINTRESOURCE(IDI_ICON_LARGE)));
    SendMessage(hWnd, WM_SETICON, ICON_SMALL, (LPARAM) LoadIcon(hInst, MAKEINTRESOURCE(IDI_ICON_SMALL)));

    // 循环保持窗口打开。
    MSG msg = {0};

    while (GetMessage(&msg, NULL, (UINT) NULL, (UINT) NULL)) {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }

    DeleteObject(hFont); // 销毁字体对象

    return 0;
}

LRESULT CALLBACK WindowProcedure(HWND hWnd, UINT msg, WPARAM wp, LPARAM lp) {
    static HBRUSH hBrush = CreateSolidBrush(RGB(255, 255, 255)); // 白色

    switch (msg) {
        case WM_COMMAND: // 控件的命令处理。
            switch (wp) {
                case SUBMIT_RNXObsRov:
                case SUBMIT_RNXObsBas:
                case SUBMIT_RNXNav:
                case SUBMIT_RTKSolPath:
                case SUBMIT_IMUPath:
                case SUBMIT_OUTPUTPath:
                case SUBMIT_ODOPath:
                    OpenFiletoTextBox(hWnd, wp);
                    break;

                case SUBMIT_CFGPath: {
                    OpenFiletoTextBox(hWnd, wp);
                    char buff[512];
                    GetWindowTextA(TextBoxCFGPath, buff, 512 * sizeof(char));
                    string cfgPath(buff);
                    if (!cfgPath.empty()) {
                        GINSOptions options(buff);
                        FillTextBoxesFromGINSOptions(options);
                    }
                    break;
                }

                case SUBMIT_Execute: {
                    if (!isRunning) {
                        char buffer[512];
                        GetWindowTextA(TextBoxCFGPath, buffer, 512 * sizeof(char));
                        string CfgPath(buffer);
                        if (CfgPath.empty()) {
                            MessageBox(hWnd, "Please at least set the config file path!", "Warning",
                                       MB_OK | MB_ICONWARNING);
                            break;
                        }

                        SetTimer(hWnd, ID_TIMER, UPDATE_INTERVAL, NULL); // 创建定时器
                        isRunning = true;
                        std::thread runThread(RunTask, hWnd);
                        runThread.detach();
                    }
                    break;
                }

                case SUBMIT_Help: {
                    MessageBox(hWnd,
                               "(i)   GPST format = 2300 120000.0\n"
                               "(ii)  The file paths must be in English\n"
                               "(iii) Go to config file for detailed settings\n"
                               "(iv)  All you could do is click the Execute button!\n"
                               "(v)   Except for the configuration file, all else could be omitted.\n"
                               "(vi)  Contact me: lewis5499@outlook.com",
                               "Help", MB_OK | MB_HELP | MB_ICONQUESTION);
                    break;
                }

                case SUBMIT_OpenLog: {
                    ChangeWorkDirToExe();
                    char currentPath[MAX_PATH];
                    if (GetCurrentDirectoryA(MAX_PATH, currentPath)) {
                        std::string logFilePath = std::string(currentPath) + "\\ginsCache.log";
                        std::ifstream logFile(logFilePath);
                        if (logFile.good()) {
                            ShellExecuteA(hWnd, "open", "notepad.exe", logFilePath.c_str(), NULL, SW_SHOW);
                        } else {
                            MessageBox(hWnd, "Log file not found.", "Error", MB_OK | MB_ICONERROR);
                        }
                        if(logFile.is_open()) {
                            logFile.close();
                        }
                    } else {
                        MessageBox(hWnd, "Failed to get current directory.", "Error", MB_OK | MB_ICONERROR);
                    }
                }

                case SUBMIT_ClearText: {
                    if (!isRunning)
                        ClearTextBox();
                    break;
                }

                default:
                    break;
            }
            break;

        case WM_TIMER:
            if (wp == ID_TIMER) {
                UpdateProgressBar(); // 定时更新进度条
            }
            break;

        case WM_HELP:
            ShellExecute(hWnd, "open", "mailto:lewis5499@outlook.com", NULL, NULL, SW_SHOWNORMAL);
            break;

        case WM_CREATE:
            AddControls(hWnd);
            break;

        case WM_CTLCOLORSTATIC:
            return (INT_PTR) hBrush;

        case WM_DESTROY:
            DeleteObject(hBrush);
            PostQuitMessage(0);
            break;

        case WM_CLOSE:
            if(GINSLogFile) {
                fclose(GINSLogFile);
            }
            DestroyWindow(hWnd);
            break;

        default:
            return DefWindowProcA(hWnd, msg, wp, lp);
    }
    return 0;
}

void AddControls(HWND hWnd) {
    // 初始化进度条
    InitCommonControls();
    hProgressBar = CreateWindowEx(0, PROGRESS_CLASS, NULL, WS_CHILD | WS_VISIBLE | PBS_SMOOTH, 50, 23, 100, 20, hWnd,
                                  (HMENU) SUBMIT_pBar, GetModuleHandle(NULL), NULL);

    // 静态文本控件
    HWND hStatic = CreateWindowExA(0, "static", "Post RTK-GNSS/INS Navigation Software",
                                   WS_VISIBLE | WS_CHILD | SS_CENTER, 150, 18, 450, 30, hWnd, NULL, NULL, NULL);
    SendMessage(hStatic, WM_SETFONT, (WPARAM) hFont, TRUE);

    // Button 'Log', 'Help', 'Clear;
    ButtonClearTextBox = CreateWindowExA(0, "button", "R", WS_VISIBLE | WS_CHILD | BS_FLAT, 15, 20, 25, 25, hWnd,
                                         (HMENU) SUBMIT_ClearText, NULL, NULL);
    SendMessage(ButtonClearTextBox, WM_SETFONT, (WPARAM) hFont, TRUE);
    ButtonHelp = CreateWindowExA(0, "button", "?", WS_VISIBLE | WS_CHILD | BS_FLAT, 650, 18, 35, 30, hWnd,
                                 (HMENU) SUBMIT_Help, NULL, NULL);
    SendMessage(ButtonHelp, WM_SETFONT, (WPARAM) hFont, TRUE);
    ButtonOpenLog = CreateWindowExA(0, "button", "Log", WS_VISIBLE | WS_CHILD | BS_FLAT, 595, 18, 40, 30, hWnd,
                                    (HMENU) SUBMIT_OpenLog, NULL, NULL);
    SendMessage(ButtonOpenLog, WM_SETFONT, (WPARAM) hFont, TRUE);

    int y_offset = 60; // 控件垂直间距的初始偏移量

    // 时间范围控件
    // GPST Start Time
    hStatic = CreateWindowExA(0, "static", "(GPST) Time Start", WS_VISIBLE | WS_CHILD | SS_LEFT, 15, y_offset, 200, 30,
                              hWnd, NULL, NULL, NULL);
    TextBoxTimeStart = CreateWindowExA(0, "edit", "", WS_VISIBLE | WS_CHILD | WS_BORDER | ES_AUTOHSCROLL, 185, y_offset,
                                       140, 30, hWnd, NULL, NULL, NULL);
    SendMessage(hStatic, WM_SETFONT, (WPARAM) hFont, TRUE);
    SendMessage(TextBoxTimeStart, WM_SETFONT, (WPARAM) hFont, TRUE);
    // GPST End Time
    hStatic = CreateWindowExA(0, "static", "Time End", WS_VISIBLE | WS_CHILD | SS_LEFT, 340, y_offset, 200, 30, hWnd,
                              NULL, NULL, NULL);
    TextBoxTimeEnd = CreateWindowExA(0, "edit", "", WS_VISIBLE | WS_CHILD | WS_BORDER | ES_AUTOHSCROLL, 440, y_offset,
                                     140, 30, hWnd, NULL, NULL, NULL);
    SendMessage(hStatic, WM_SETFONT, (WPARAM) hFont, TRUE);
    SendMessage(TextBoxTimeEnd, WM_SETFONT, (WPARAM) hFont, TRUE);
    // 程序执行控件
    // Execute button
    ButtonExecute = CreateWindowExA(0, "button", "Execute", WS_VISIBLE | WS_CHILD | BS_FLAT, 595, y_offset, 90, 30,
                                    hWnd, (HMENU) SUBMIT_Execute, NULL, NULL);
    SendMessage(ButtonExecute, WM_SETFONT, (WPARAM) hFont, TRUE);
    y_offset += 42; // 更新垂直偏移量

    // RINEX OBS: Rover
    hStatic = CreateWindowExA(0, "static", "RINEX OBS: Rover", WS_VISIBLE | WS_CHILD | SS_LEFT, 15, y_offset, 270, 30,
                              hWnd, NULL, NULL, NULL);
    TextBoxRNXObsRov = CreateWindowExA(0, "edit", "", WS_VISIBLE | WS_CHILD | WS_BORDER | ES_AUTOHSCROLL, 185, y_offset,
                                       450, 30, hWnd, NULL, NULL, NULL);
    ButtonRNXObsRov  = CreateWindowExA(0, "button", "...", WS_VISIBLE | WS_CHILD | BS_FLAT, 650, y_offset, 35, 30, hWnd,
                                       (HMENU) SUBMIT_RNXObsRov, NULL, NULL);
    SendMessage(hStatic, WM_SETFONT, (WPARAM) hFont, TRUE);
    SendMessage(TextBoxRNXObsRov, WM_SETFONT, (WPARAM) hFont, TRUE);
    SendMessage(ButtonRNXObsRov, WM_SETFONT, (WPARAM) hFont, TRUE);
    y_offset += 42; // 更新垂直偏移量

    // RINEX OBS: Base
    hStatic = CreateWindowExA(0, "static", "RINEX OBS: Base", WS_VISIBLE | WS_CHILD | SS_LEFT, 15, y_offset, 270, 30,
                              hWnd, NULL, NULL, NULL);
    TextBoxRNXObsBas = CreateWindowExA(0, "edit", "", WS_VISIBLE | WS_CHILD | WS_BORDER | ES_AUTOHSCROLL, 185, y_offset,
                                       450, 30, hWnd, NULL, NULL, NULL);
    ButtonRNXObsBas  = CreateWindowExA(0, "button", "...", WS_VISIBLE | WS_CHILD | BS_FLAT, 650, y_offset, 35, 30, hWnd,
                                       (HMENU) SUBMIT_RNXObsBas, NULL, NULL);
    SendMessage(hStatic, WM_SETFONT, (WPARAM) hFont, TRUE);
    SendMessage(TextBoxRNXObsBas, WM_SETFONT, (WPARAM) hFont, TRUE);
    SendMessage(ButtonRNXObsBas, WM_SETFONT, (WPARAM) hFont, TRUE);
    y_offset += 42; // 更新垂直偏移量

    // RINEX NAV
    hStatic = CreateWindowExA(0, "static", "RINEX NAV: Any", WS_VISIBLE | WS_CHILD | SS_LEFT, 15, y_offset, 270, 30,
                              hWnd, NULL, NULL, NULL);
    TextBoxRNXNav = CreateWindowExA(0, "edit", "", WS_VISIBLE | WS_CHILD | WS_BORDER | ES_AUTOHSCROLL, 185, y_offset,
                                    450, 30, hWnd, NULL, NULL, NULL);
    ButtonRNXNav  = CreateWindowExA(0, "button", "...", WS_VISIBLE | WS_CHILD | BS_FLAT, 650, y_offset, 35, 30, hWnd,
                                    (HMENU) SUBMIT_RNXNav, NULL, NULL);
    SendMessage(hStatic, WM_SETFONT, (WPARAM) hFont, TRUE);
    SendMessage(TextBoxRNXNav, WM_SETFONT, (WPARAM) hFont, TRUE);
    SendMessage(ButtonRNXNav, WM_SETFONT, (WPARAM) hFont, TRUE);
    y_offset += 42; // 更新垂直偏移量

    // RTK Solution Path
    hStatic = CreateWindowExA(0, "static", "GNSS RTK Path", WS_VISIBLE | WS_CHILD | SS_LEFT, 15, y_offset, 270, 30,
                              hWnd, NULL, NULL, NULL);
    TextBoxRTKSolPath = CreateWindowExA(0, "edit", "", WS_VISIBLE | WS_CHILD | WS_BORDER | ES_AUTOHSCROLL, 185,
                                        y_offset, 450, 30, hWnd, NULL, NULL, NULL);
    ButtonRTKSolPath = CreateWindowExA(0, "button", "...", WS_VISIBLE | WS_CHILD | BS_FLAT, 650, y_offset, 35, 30, hWnd,
                                       (HMENU) SUBMIT_RTKSolPath, NULL, NULL);
    SendMessage(hStatic, WM_SETFONT, (WPARAM) hFont, TRUE);
    SendMessage(TextBoxRTKSolPath, WM_SETFONT, (WPARAM) hFont, TRUE);
    SendMessage(ButtonRTKSolPath, WM_SETFONT, (WPARAM) hFont, TRUE);
    y_offset += 42; // 更新垂直偏移量

    // IMU Data Path
    hStatic = CreateWindowExA(0, "static", "INS IMU Path", WS_VISIBLE | WS_CHILD | SS_LEFT, 15, y_offset, 270, 30, hWnd,
                              NULL, NULL, NULL);
    TextBoxIMUPath = CreateWindowExA(0, "edit", "", WS_VISIBLE | WS_CHILD | WS_BORDER | ES_AUTOHSCROLL, 185, y_offset,
                                     450, 30, hWnd, NULL, NULL, NULL);
    ButtonIMUPath  = CreateWindowExA(0, "button", "...", WS_VISIBLE | WS_CHILD | BS_FLAT, 650, y_offset, 35, 30, hWnd,
                                     (HMENU) SUBMIT_IMUPath, NULL, NULL);
    SendMessage(hStatic, WM_SETFONT, (WPARAM) hFont, TRUE);
    SendMessage(TextBoxIMUPath, WM_SETFONT, (WPARAM) hFont, TRUE);
    SendMessage(ButtonIMUPath, WM_SETFONT, (WPARAM) hFont, TRUE);
    y_offset += 42; // 更新垂直偏移量

    // ODO Data Path
    hStatic = CreateWindowExA(0, "static", "Odometer Path", WS_VISIBLE | WS_CHILD | SS_LEFT, 15, y_offset, 270, 30,
                              hWnd, NULL, NULL, NULL);
    TextBoxODOPath = CreateWindowExA(0, "edit", "", WS_VISIBLE | WS_CHILD | WS_BORDER | ES_AUTOHSCROLL, 185, y_offset,
                                     450, 30, hWnd, NULL, NULL, NULL);
    ButtonODOPath  = CreateWindowExA(0, "button", "...", WS_VISIBLE | WS_CHILD | BS_FLAT, 650, y_offset, 35, 30, hWnd,
                                     (HMENU) SUBMIT_ODOPath, NULL, NULL);
    SendMessage(hStatic, WM_SETFONT, (WPARAM) hFont, TRUE);
    SendMessage(TextBoxODOPath, WM_SETFONT, (WPARAM) hFont, TRUE);
    SendMessage(ButtonODOPath, WM_SETFONT, (WPARAM) hFont, TRUE);
    y_offset += 42; // 更新垂直偏移量

    // OUTPUT Path
    hStatic = CreateWindowExA(0, "static", "Output Path", WS_VISIBLE | WS_CHILD | SS_LEFT, 15, y_offset, 270, 30, hWnd,
                              NULL, NULL, NULL);
    TextBoxOUTPUTPath = CreateWindowExA(0, "edit", "", WS_VISIBLE | WS_CHILD | WS_BORDER | ES_AUTOHSCROLL, 185,
                                        y_offset, 450, 30, hWnd, NULL, NULL, NULL);
    ButtonOUTPUTPath = CreateWindowExA(0, "button", "...", WS_VISIBLE | WS_CHILD | BS_FLAT, 650, y_offset, 35, 30, hWnd,
                                       (HMENU) SUBMIT_OUTPUTPath, NULL, NULL);
    SendMessage(hStatic, WM_SETFONT, (WPARAM) hFont, TRUE);
    SendMessage(TextBoxOUTPUTPath, WM_SETFONT, (WPARAM) hFont, TRUE);
    SendMessage(ButtonOUTPUTPath, WM_SETFONT, (WPARAM) hFont, TRUE);
    y_offset += 42; // 更新垂直偏移量

    // CFG Data Path
    hStatic = CreateWindowExA(0, "static", "Config Path", WS_VISIBLE | WS_CHILD | SS_LEFT, 15, y_offset, 270, 30, hWnd,
                              NULL, NULL, NULL);
    TextBoxCFGPath = CreateWindowExA(0, "edit", "", WS_VISIBLE | WS_CHILD | WS_BORDER | ES_AUTOHSCROLL, 185, y_offset,
                                     450, 30, hWnd, NULL, NULL, NULL);
    ButtonCFGPath  = CreateWindowExA(0, "button", "...", WS_VISIBLE | WS_CHILD | BS_FLAT, 650, y_offset, 35, 30, hWnd,
                                     (HMENU) SUBMIT_CFGPath, NULL, NULL);
    SendMessage(hStatic, WM_SETFONT, (WPARAM) hFont, TRUE);
    SendMessage(TextBoxCFGPath, WM_SETFONT, (WPARAM) hFont, TRUE);
    SendMessage(ButtonCFGPath, WM_SETFONT, (WPARAM) hFont, TRUE);
}

void EnableDPIAwareness() {
    HMODULE user32 = LoadLibraryA("User32.dll");
    if (user32) {
        typedef BOOL(WINAPI * SetProcessDPIAwareFunc)(void);
        auto setProcessDPIAware = (SetProcessDPIAwareFunc) GetProcAddress(user32, "SetProcessDPIAware");
        if (setProcessDPIAware) {
            setProcessDPIAware();
        }
        FreeLibrary(user32);
    }
}

// Helper function to format the exception information
void FormatExceptionInfo(EXCEPTION_POINTERS *ExceptionInfo, char *buffer, size_t bufferSize) {
    if (ExceptionInfo == NULL || buffer == NULL || bufferSize == 0) {
        return;
    }

    // Extract exception code and address
    DWORD exceptionCode    = ExceptionInfo->ExceptionRecord->ExceptionCode;
    PVOID exceptionAddress = ExceptionInfo->ExceptionRecord->ExceptionAddress;

    // Format the message
    StringCchPrintf(buffer, bufferSize,
                    "Process: RTK and GNSS/INS encountered a serious error!\n\n"
                    "Exception Code: 0x%08X\n"
                    "Exception Address: 0x%p",
                    exceptionCode, exceptionAddress);
}

LONG WINAPI CustomUnhandledExceptionFilter(EXCEPTION_POINTERS *ExceptionInfo) {
    char messageBuffer[512];
    FormatExceptionInfo(ExceptionInfo, messageBuffer, sizeof(messageBuffer));
    MessageBox(NULL, messageBuffer, "Error", MB_OK | MB_ICONERROR);
    isRunning = false;
    return EXCEPTION_EXECUTE_HANDLER;
}

int OpenFiletoTextBox(HWND hWnd, WPARAM wp) {
    OPENFILENAMEA ofn;
    char szFile[512];
    ZeroMemory(&ofn, sizeof(ofn));
    ofn.lStructSize     = sizeof(ofn);
    ofn.hwndOwner       = hWnd;
    ofn.lpstrFile       = szFile;
    ofn.lpstrFile[0]    = '\0';
    ofn.nMaxFile        = sizeof(szFile);
    ofn.lpstrFilter     = "All Files\0*.*\0";
    ofn.nFilterIndex    = 1;
    ofn.lpstrFileTitle  = NULL;
    ofn.nMaxFileTitle   = 0;
    ofn.lpstrInitialDir = NULL;
    ofn.Flags           = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

    if (GetOpenFileNameA(&ofn) == TRUE) {
        switch (wp) {
            case SUBMIT_RNXObsRov:
                SetWindowTextA(TextBoxRNXObsRov, ofn.lpstrFile);
                break;
            case SUBMIT_RNXObsBas:
                SetWindowTextA(TextBoxRNXObsBas, ofn.lpstrFile);
                break;
            case SUBMIT_RNXNav:
                SetWindowTextA(TextBoxRNXNav, ofn.lpstrFile);
                break;
            case SUBMIT_RTKSolPath:
                SetWindowTextA(TextBoxRTKSolPath, ofn.lpstrFile);
                break;
            case SUBMIT_IMUPath:
                SetWindowTextA(TextBoxIMUPath, ofn.lpstrFile);
                break;
            case SUBMIT_OUTPUTPath:
                SetWindowTextA(TextBoxOUTPUTPath, ofn.lpstrFile);
                break;
            case SUBMIT_ODOPath:
                SetWindowTextA(TextBoxODOPath, ofn.lpstrFile);
                break;
            case SUBMIT_CFGPath:
                SetWindowTextA(TextBoxCFGPath, ofn.lpstrFile);
                break;
            default:
                break;
        }
    }
    return 0;
}

void ParseFileNameFromTextBox(BasicParams &fps) {
    char buffer[512];

    GetWindowTextA(TextBoxTimeStart, buffer, sizeof(buffer));
    fps.timeStart = buffer;
    if (fps.timeStart == "auto")
        fps.timeStart = "0 0.0";

    GetWindowTextA(TextBoxTimeEnd, buffer, sizeof(buffer));
    fps.timeEnd = buffer;
    if (fps.timeEnd == "auto")
        fps.timeEnd = "inf inf";

    GetWindowTextA(TextBoxRNXObsRov, buffer, sizeof(buffer));
    fps.rnxObsRov = buffer;

    GetWindowTextA(TextBoxRNXObsBas, buffer, sizeof(buffer));
    fps.rnxObsBas = buffer;

    GetWindowTextA(TextBoxRNXNav, buffer, sizeof(buffer));
    fps.rnxNav = buffer;

    GetWindowTextA(TextBoxRTKSolPath, buffer, sizeof(buffer));
    fps.rtkSolPath = buffer;

    GetWindowTextA(TextBoxIMUPath, buffer, sizeof(buffer));
    fps.imuPath = buffer;

    GetWindowTextA(TextBoxOUTPUTPath, buffer, sizeof(buffer));
    fps.outputPath = buffer;

    GetWindowTextA(TextBoxODOPath, buffer, sizeof(buffer));
    fps.odoPath = buffer;

    GetWindowTextA(TextBoxCFGPath, buffer, sizeof(buffer));
    fps.cfgPath = buffer;

    if (!fps.cfgPath.empty())
        setConfig = true;
}

void FillTextBoxesFromGINSOptions(const GINSOptions &opts) {
    char buff[512];
    int len;

    len = GetWindowTextA(TextBoxTimeStart, buff, 512 * sizeof(char));
    if (fabs(opts.getStartTow()) < 1e-5 && opts.getStartWeek() == 0 && len == 0) {
        SetWindowTextA(TextBoxTimeStart, "auto");
    } else if (len == 0) {
        SetWindowTextA(TextBoxTimeStart,
                       (to_string(opts.getStartWeek()) + " " + to_string(opts.getStartTow())).c_str());
    } else
        ;

    len = GetWindowTextA(TextBoxTimeEnd, buff, 512 * sizeof(char));
    if (opts.getEndWeek() == cmn::infWeek && fabs(opts.getEndTow() - cmn::infTow) < 1e-5 && len == 0) {
        SetWindowTextA(TextBoxTimeEnd, "auto");
    } else if (len == 0) {
        SetWindowTextA(TextBoxTimeEnd, (to_string(opts.getEndWeek()) + " " + to_string(opts.getEndTow())).c_str());
    } else
        ;

    if (!GetWindowTextA(TextBoxRNXObsRov, buff, 512 * sizeof(char))) {
        SetWindowTextA(TextBoxRNXObsRov, opts.getRnxRovObspath().c_str());
    }

    if (!GetWindowTextA(TextBoxRNXObsBas, buff, 512 * sizeof(char))) {
        SetWindowTextA(TextBoxRNXObsBas, opts.getRnxBasObspath().c_str());
    }

    if (!GetWindowTextA(TextBoxRNXNav, buff, 512 * sizeof(char))) {
        SetWindowTextA(TextBoxRNXNav, opts.getRnxNaviPath().c_str());
    }

    if (!GetWindowTextA(TextBoxRTKSolPath, buff, 512 * sizeof(char))) {
        SetWindowTextA(TextBoxRTKSolPath, opts.getGnssFilePath().c_str());
    }

    if (!GetWindowTextA(TextBoxIMUPath, buff, 512 * sizeof(char))) {
        SetWindowTextA(TextBoxIMUPath, opts.getImuFilePath().c_str());
    }

    if (!GetWindowTextA(TextBoxOUTPUTPath, buff, 512 * sizeof(char))) {
        SetWindowTextA(TextBoxOUTPUTPath, opts.getOutputPath().c_str());
    }

    if (!GetWindowTextA(TextBoxODOPath, buff, 512 * sizeof(char))) {
        SetWindowTextA(TextBoxODOPath, opts.getOdoFilePath().c_str());
    }
}

void ClearTextBox() {
    SetWindowTextA(TextBoxTimeStart, "");
    SetWindowTextA(TextBoxTimeEnd, "");
    SetWindowTextA(TextBoxRNXObsRov, "");
    SetWindowTextA(TextBoxRNXObsBas, "");
    SetWindowTextA(TextBoxRNXNav, "");
    SetWindowTextA(TextBoxRTKSolPath, "");
    SetWindowTextA(TextBoxIMUPath, "");
    SetWindowTextA(TextBoxOUTPUTPath, "");
    SetWindowTextA(TextBoxODOPath, "");
    SetWindowTextA(TextBoxCFGPath, "");
}

void RedirectStdOutErrToLog(FILE *logFile) {
    // Open the log file in append mode
    logFile = fopen("ginsCache.log", "w");
    if (logFile == NULL) {
        fprintf(stderr, "Error opening log file.\n");
    }
    // Redirect stdout and stderr to the log file
    if (dup2(fileno(logFile), STDOUT_FILENO) == -1) {
        fprintf(stderr, "Error redirecting stdout to log file.\n");
        fclose(logFile);
    }
    if (dup2(fileno(logFile), STDERR_FILENO) == -1) {
        fprintf(stderr, "Error redirecting stderr to log file.\n");
        fclose(logFile);
    }
}

int ChangeWorkDirToExe() {
    char path[MAX_PATH];
    // Get the full path of the current executable
    DWORD length = GetModuleFileName(NULL, path, MAX_PATH);
    if (length == 0) {
        perror("[Error]: Unable to get current executable path");
        return -1;
    }

    // Extract the directory from the full path
    char *lastSlash = strrchr(path, '\\');
    if (lastSlash != NULL) {
        *lastSlash = '\0'; // Null-terminate the string at the last slash
        // Change the working directory to the executable's directory
        if (SetCurrentDirectory(path) != 0) {
            // printf("Working directory changed to the executable's directory\n");
        } else {
            perror("[Error]: Unable to change working directory");
            return -1;
        }
    } else {
        perror("[Error]: Unable to extract directory from executable path");
        return -1;
    }

    return 0;
}

void UpdateProgressBar() {
    if (!bReverse) {
        step++;
        if (step >= 100) {
            step     = 100;
            bReverse = TRUE;
        }
    } else {
        step--;
        if (step <= 0) {
            step     = 0;
            bReverse = FALSE;
        }
    }
    SendMessage(hProgressBar, PBM_SETPOS, step, 0); // 更新进度条
}

void RunTask(HWND hWnd) {
    SetUnhandledExceptionFilter(CustomUnhandledExceptionFilter);

    try {
        ParseFileNameFromTextBox(params);

        if (setConfig) {
            MessageBox(hWnd,
                       " Execute: RTK and GNSS/INS Start Processing!\n"
                       " Do not Close the Program Window until receive new MsgBox.",
                       "Message", MB_OK | MB_ICONWARNING);

            ChangeWorkDirToExe();
            RedirectStdOutErrToLog(GINSLogFile);

            GINSOptions giOpts(params.cfgPath.c_str());
            giOpts.RenewOptions(params);
            Run(giOpts);

        } else {
            isRunning = false;
            KillTimer(hWnd, ID_TIMER);
            step = 0;
            SendMessage(hProgressBar, PBM_SETPOS, step, 0);
            MessageBox(hWnd, "Please at least set the config file path!", "Warning", MB_OK | MB_ICONWARNING);
            return;
        }

    } catch (const std::exception &e) {
        isRunning = false;
        KillTimer(hWnd, ID_TIMER);
        step = 0;
        SendMessage(hProgressBar, PBM_SETPOS, step, 0);
        MessageBox(hWnd, e.what(), "Standard Exception", MB_OK | MB_ICONERROR);
        return;
    } catch (...) {
        isRunning = false;
        KillTimer(hWnd, ID_TIMER);
        step = 0;
        SendMessage(hProgressBar, PBM_SETPOS, step, 0);
        MessageBox(hWnd, "An unknown error occurred!", "Error", MB_OK | MB_ICONERROR);
        return;
    }

    isRunning = false;
    KillTimer(hWnd, ID_TIMER);
    step = 0;
    SendMessage(hProgressBar, PBM_SETPOS, step, 0);
    MessageBox(hWnd, "Process: RTK and GNSS/INS have been done.\n", "Message", MB_OK | MB_ICONINFORMATION);
}

#endif

int Run(GINSOptions &options) {
    std::cout << _PROJECT_HEADER_ << endl;

    /* --------------------------------------------- create thread pool --------------------------------------------- */
    ThreadPool::synced_stream syncOut;
    ThreadPool::thread_pool threadPool;

    // GNSS RTK positioning
    threadPool.detach_task([&options, &syncOut] {
        try {
            RTKEntry::RunGnssRTK(params.cfgPath, options.getGnssFilePath(), options.getRnxRovObspath(),
                                 options.getRnxBasObspath(), options.getRnxNaviPath());
        } catch (const std::exception &e) {
            syncOut.println("[ERROR]: GNSS RTK thread exception: ", e.what());
        } catch (...) {
            syncOut.println("[ERROR]: GNSS RTK thread unknown exception.");
        }
    });

    // IMU Initial Alignment
    double initAtt[3] = {0.0, 0.0, 0.0};
    threadPool.detach_task([&options, &syncOut, &initAtt] {
        auto start = std::chrono::high_resolution_clock::now();
        try {
            ImuFileLoader imuStaticAlignment(cmn::ImuFormatType(options.getImuDataFormat()), options.getImuFilePath(),
                                             options.getImuCoordType(), options.getImuStartWeek(), options.getImuSamplingRate(),
                                             options.getImuInitStaticTime(), true);
            imuStaticAlignment.roughAlign();
            imuStaticAlignment.getRPY(initAtt);
        } catch (const std::exception &e) {
            syncOut.println("[ERROR]: IMU alignment thread exception: ", e.what());
        } catch (...) {
            syncOut.println("[ERROR]: IMU alignment thread unknown exception.");
        }
        auto end                               = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end - start;
        syncOut.println("[INFO]:  IMU alignment thread execution time: ", duration.count(), " seconds.");
    });

    // wait all tasks to complete
    std::cout << "[Thread Pool]: Waiting for the " << threadPool.get_tasks_running() << " running tasks to complete."
              << std::endl;
    threadPool.wait();
    std::cout << "[Thread Pool]: All running tasks completed. " << threadPool.get_tasks_queued()
              << " tasks still queued." << std::endl;

    /* -------------------------------------------- data pre-processing --------------------------------------------- */
    // Open Gnss and Imu files
    GnssFileLoader gnssFile(options.getGnssFilePath(), false);
    ImuFileLoader imuFile(cmn::ImuFormatType(options.getImuDataFormat()), options.getImuFilePath(),
                          options.getImuCoordType(), options.getImuStartWeek(), options.getImuSamplingRate(), options.getImuInitStaticTime(),
                          false);

    // renew and print gins options: check process time, data alignment and renew initstate
    options.RenewOptions(imuFile, gnssFile, vector<double>(initAtt, initAtt + 3));
    auto imuData = imuFile.getImuEpochs(false);

    // renew and print gins options: check process time, data alignment and renew initstate
    options.RenewOptions(imuFile, gnssFile, vector<double>(initAtt, initAtt + 3));
    options.printOptions();

    // setup output path
    NaviFileSaver navResFile(options.getOutputPath() + "nav_result.pos", FileMode::TEXT);
    NaviFileSaver imuErrFile(options.getOutputPath() + "imu_error.dat", FileMode::TEXT);
    imuErrFile.writeHeaders(cmn::imuErrorHeaders);

    // create variables for GINS MAIN and PLOTTER
    Time endTime((std::uint16_t) options.getEndWeek(), options.getEndTow());
    int RANK = options.getUseImuScale()?GINSManager::RANK_21:GINSManager::RANK_15;
    std::vector<double> t, lat, lon, height, vn, ve, vd, roll, pitch, yaw;
    std::vector<std::array<double, 3>> std_gyroBias, std_acceBias, std_gyroScale, std_acceScale, gyroBias, acceBias, gyroScale, acceScale;
    t.reserve(imuData.size());
    lat.reserve(imuData.size());
    lon.reserve(imuData.size());
    height.reserve(imuData.size());
    vn.reserve(imuData.size());
    ve.reserve(imuData.size());
    vd.reserve(imuData.size());
    roll.reserve(imuData.size());
    pitch.reserve(imuData.size());
    yaw.reserve(imuData.size());
    std_gyroBias.reserve(imuData.size());
    std_acceBias.reserve(imuData.size());
    std_gyroScale.reserve(imuData.size());
    std_acceScale.reserve(imuData.size());
    gyroBias.reserve(imuData.size());
    acceBias.reserve(imuData.size());
    gyroScale.reserve(imuData.size());
    acceScale.reserve(imuData.size());

    /* -------------------------------------------- save raw imu data ----------------------------------------------- */
    /*
     * If a variable is read-only and will not be modified in any thread,
     * it is usually safe for multiple threads to read it simultaneously.
     * This is because the read operation itself is atomic and will not cause race conditions.
     */
    threadPool.detach_task([&syncOut, &imuData, &options] {
        try {
            // save raw imu data to 'txt' file
            NaviFileSaver::saveRawImuToTxt(imuData, options.getOutputPath());
        } catch (const std::exception &e) {
            syncOut.println("[ERROR]: IMU raw data saving thread exception: ", e.what());
        } catch (...) {
            syncOut.println("[ERROR]: IMU raw data saving thread unknown exception.");
        }
    });

    /* ------------------------------------------------ GINS main --------------------------------------------------- */
    GINSManager ginsManager(options);
    Time timestamp;
    NavState navstate;
    MatrixXd covariance = Eigen::MatrixXd::Zero(RANK,RANK);
    GNSS gnss           = gnssFile.getCurrentGNSS();
    IMU imu             = imuFile.getCurrentIMU();

    // add imudata to GINSManager and compensate IMU error
    ginsManager.addImuData(imu, true);

    // add gnssdata to GINSManager
    ginsManager.addGnssData(gnss);

    // Set the initial zero linear/angular vel update time and the odometer/nonholonomic constraint update time
    ginsManager.setInitUpateTime();

    // Looping through data
    int imuindex = 1; // now is the second imu to be added
    while (true) {
        // load new gnssdata when current state time is newer than GNSS time and add it to GINSManager
        if (gnss.time < imu.time && !gnssFile.isEOF()) {
            gnss = gnssFile.getNextGNSS();
            ginsManager.addGnssData(gnss);
        }

        // load new imudata and add it to GINSManager
        imu = imuFile.getNextIMU();
        if (imu.time > endTime || imuFile.isEOF()) {
            break;
        }
        imu.tag = GINSManager::detectZUPT(imuData, imuindex, navstate.pos, 20);
        ginsManager.addImuData(imu);

        // process new imudata
        ginsManager.newImuProcess();

        // get current timestamp, navigation state and covariance
        timestamp  = ginsManager.getTimestamp();
        navstate   = ginsManager.getNavState();
        covariance = ginsManager.getCovariance();

        // save processing results
        ginsManager.writeResult(navResFile, imuErrFile, navstate, timestamp, covariance);
        if (options.getUsePlotResults()) {
            t.push_back(imuindex);
            lat.push_back(navstate.pos(0)*R2D);
            lon.push_back(navstate.pos(1)*R2D);
            height.push_back(navstate.pos(2));
            vn.push_back(navstate.vel(0));
            ve.push_back(navstate.vel(1));
            vd.push_back(navstate.vel(2));
            roll.push_back(navstate.att.euler(0)*R2D);
            pitch.push_back(navstate.att.euler(1)*R2D);
            yaw.push_back(navstate.att.euler(2)*R2D);
            std_gyroBias.push_back({SQRT_(covariance(9, 9)) * R2D * 3600, SQRT_(covariance(10, 10)) * R2D * 3600, SQRT_(covariance(11, 11)) * R2D * 3600});
            std_acceBias.push_back({SQRT_(covariance(12, 12)) * 1e5, SQRT_(covariance(13, 13)) * 1e5, SQRT_(covariance(14, 14)) * 1e5,});
            gyroBias.push_back({navstate.imuerror.gyrbias(0) * R2D * 3600, navstate.imuerror.gyrbias(1) * R2D * 3600, navstate.imuerror.gyrbias(2) * R2D * 3600});
            acceBias.push_back({navstate.imuerror.accbias(0) * 1e5, navstate.imuerror.accbias(1) * 1e5, navstate.imuerror.accbias(2) * 1e5});
            if(options.getUseImuScale()) {
                std_gyroScale.push_back({SQRT_(covariance(15, 15)) * 1e6, SQRT_(covariance(16, 16)) * 1e6, SQRT_(covariance(17, 17)) * 1e6});
                std_acceScale.push_back({SQRT_(covariance(18, 18)) * 1e6, SQRT_(covariance(19, 19)) * 1e6, SQRT_(covariance(20, 20)) * 1e6});
                gyroScale.push_back({navstate.imuerror.gyrscale(0) * 1e6, navstate.imuerror.gyrscale(1) * 1e6, navstate.imuerror.gyrscale(2) * 1e6});
                acceScale.push_back({navstate.imuerror.accscale(0) * 1e6, navstate.imuerror.accscale(1) * 1e6, navstate.imuerror.accscale(2) * 1e6});
            }
        }

        ++imuindex;
    }

    /* ---------------------------------------------------- plot ---------------------------------------------------- */
    if (options.getUsePlotResults()) {
        Plotter plotter(t,lat,lon,height,vn,ve,vd,roll,pitch,yaw,
                        gyroBias,acceBias,gyroScale,acceScale,
                        std_gyroBias,std_acceBias,std_gyroScale,std_acceScale,
                        options.getOutputPath(),syncOut,threadPool);
        plotter.plotAll();
        plotter.showAll();
    }

    return 1;
}
