
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

typedef unsigned char Byte;
using namespace std;

Byte* Smoothing(Byte* input, int width, int height) {
    Byte* output = new Byte[width * height];
    for (int y = 1; y < height - 1; y++) {
        for (int x = 1; x < width - 1; x++) {
            int sum = 0;
            for (int ky = -1; ky <= 1; ky++)
                for (int kx = -1; kx <= 1; kx++)
                    sum += input[(y + ky) * width + (x + kx)];
            output[y * width + x] = sum / 9;
        }
    }
    return output;
}

int int_sqrt(int x) {
    int res = 0;
    for (int i = 0; i * i <= x; ++i) res = i;
    return res;
}

//Açýyý yaklaþýk hesaplama
double atan2_approx(double y, double x) {
    if (x == 0.0) return (y > 0.0 ? 1.5708 : -1.5708);
    double atan, z = y / x;
    if (z < 0) z = -z;
    if (z < 1.0) atan = z / (1.0 + 0.28 * z * z);
    else atan = 1.5708 - z / (z * z + 0.28);
    if (x < 0.0) return (y < 0.0) ? -(3.1416 - atan) : (3.1416 - atan);
    return (y < 0.0) ? -atan : atan; 
}

//Görüntünün her pikselinde : Kenar þiddetini ve Kenar yönünü hesaplar.
//Sobel filtresi ile yapýlýr

int* Gradient(Byte* raw, int w, int h, double*& angles) {
    int* grad = new int[w * h]; angles = new double[w * h];
    int Gx[3][3] = { {-1,0,1}, {-2,0,2}, {-1,0,1} }, Gy[3][3] = { {-1,-2,-1}, {0,0,0}, {1,2,1} };
    for (int y = 1; y < h - 1; y++)
        for (int x = 1; x < w - 1; x++) {
            int sx = 0, sy = 0;
            for (int ky = -1; ky <= 1; ky++)
                for (int kx = -1; kx <= 1; kx++) {
                    int val = raw[(y + ky) * w + (x + kx)];
                    sx += Gx[ky + 1][kx + 1] * val;
                    sy += Gy[ky + 1][kx + 1] * val;
                }
            grad[y * w + x] = int_sqrt(sx * sx + sy * sy);
            angles[y * w + x] = atan2_approx(sy, sx);
        }
    return grad;
}

Byte* CannyEdge(int* grad, int w, int h, double* ang) {
    Byte* strongEdges = new Byte[w * h];
    Byte* weakEdges = new Byte[w * h];
    for (int i = 0; i < w * h; i++) {
        strongEdges[i] = 0;
        weakEdges[i] = 0;
    }

    const int lowThreshold = 50;
    const int highThreshold = 100;

    //non maximum threshold
    for (int y = 1; y < h - 1; y++) {
        for (int x = 1; x < w - 1; x++) {
            double a = ang[y * w + x] * 180.0 / 3.1416;
            if (a < 0) a += 180;
            int cur = grad[y * w + x], prev = 0, next = 0;

            if ((a >= 0 && a < 22.5) || (a >= 157.5 && a <= 180)) {
                prev = grad[y * w + x - 1];
                next = grad[y * w + x + 1];
            }
            else if (a >= 22.5 && a < 67.5) {
                prev = grad[(y - 1) * w + x + 1];
                next = grad[(y + 1) * w + x - 1];
            }
            else if (a >= 67.5 && a < 112.5) {
                prev = grad[(y - 1) * w + x];
                next = grad[(y + 1) * w + x];
            }
            else {
                prev = grad[(y - 1) * w + x - 1];
                next = grad[(y + 1) * w + x + 1];
            }

            if (cur >= prev && cur >= next) {
                if (cur >= highThreshold) strongEdges[y * w + x] = 255;
                else if (cur >= lowThreshold) weakEdges[y * w + x] = 255;
            }
        }
    }

    for (int y = 1; y < h - 1; y++) {
        for (int x = 1; x < w - 1; x++) {
            if (weakEdges[y * w + x]) {
                for (int dy = -1; dy <= 1; dy++)
                    for (int dx = -1; dx <= 1; dx++)
                        if (strongEdges[(y + dy) * w + (x + dx)])
                            strongEdges[y * w + x] = 255;
            }
        }
    }

    delete[] weakEdges;
    return strongEdges;
}

Byte* CreateColorBuffer(Byte* gray, int w, int h) {
    Byte* color = new Byte[w * h * 3];
    for (int i = 0; i < w * h; i++) {
        color[i * 3 + 0] = gray[i];
        color[i * 3 + 1] = gray[i];
        color[i * 3 + 2] = gray[i];
    }
    return color;
}

void DrawLine(Byte* img, int w, int h, int x0, int y0, int x1, int y1, Byte r, Byte g, Byte b) {
    int dx = abs(x1 - x0), dy = abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1, err = dx - dy;

    while (true) {
        if (x0 >= 0 && x0 < w && y0 >= 0 && y0 < h) {
            int idx = (y0 * w + x0) * 3;
            img[idx + 0] = b;
            img[idx + 1] = g;
            img[idx + 2] = r;
        }
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx) { err += dx; y0 += sy; }
    }
}

void DrawCircle(Byte* img, int w, int h, int cx, int cy, int radius, Byte r, Byte g, Byte b) {
    for (int angle = 0; angle < 360; angle++) {
        double rad = angle * 3.1416 / 180.0;
        int x = int(cx + radius * cos(rad)); //daire çevre noktalarý hesaplanýr.
        int y = int(cy + radius * sin(rad));
        if (x >= 0 && x < w && y >= 0 && y < h) { //hesaplanan nokta 
            int idx = (y * w + x) * 3;
            img[idx + 0] = b; 
            img[idx + 1] = g;
            img[idx + 2] = r;
        }
    }
}
//r = x * cos(?) + y * sin(?)
int* Hough_Line(Byte* edge, int w, int h, int& hw, int& hh) {
    hw = 180;
    hh = 2 * int_sqrt(w * w + h * h);
    int* acc = new int[hw * hh];
    for (int i = 0; i < hw * hh; i++) acc[i] = 0;
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            if (edge[y * w + x] > 250) {
                for (int t = 0; t < 180; t++) {
                    double rad = t * 3.1416 / 180.0;
                    int r = int(x * cos(rad) + y * sin(rad)) + hh / 2;
                    if (r >= 0 && r < hh) acc[t * hh + r]++;
                }
            }
        }
    }
    return acc;
}

int* Hough_Circle(Byte* edge, int w, int h, int radius, int& cw, int& ch) {
    cw = w; ch = h;
    int* acc = new int[w * h];
    for (int i = 0; i < w * h; i++) acc[i] = 0;
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            if (edge[y * w + x] > 250) {
                for (int a = 0; a < 360; a++) {
                    double rad = a * 3.1416 / 180.0;
                    int cx = int(x - radius * cos(rad));
                    int cy = int(y - radius * sin(rad));
                    if (cx >= 0 && cx < w && cy >= 0 && cy < h)
                        acc[cy * w + cx]++;
                }
            }
        }
    }
    return acc;
}

void DrawSelectedLines(int* H, int hw, int hh, Byte* img, int w, int h, int count) {
    struct L { int t, r, v; } best[100];
    for (int i = 0; i < count; i++) best[i] = { 0, 0, 0 };
    for (int t = 0; t < hw; t++) for (int r = 0; r < hh; r++) {
        int v = H[t * hh + r];
        for (int i = 0; i < count; i++) {
            if (v > best[i].v) {
                for (int j = count - 1; j > i; j--) best[j] = best[j - 1];
                best[i] = { t, r, v };
                break;
            }
        }
    }

    for (int i = 0; i < count; i++) {
        double theta = best[i].t * 3.1416 / 180.0;
        double a = cos(theta), b = sin(theta), r = best[i].r - hh / 2;
        int lineLength = std::max(w, h);
        int x0 = int(r * a + lineLength * -b);
        int y0 = int(r * b + lineLength * a);
        int x1 = int(r * a - lineLength * -b);
        int y1 = int(r * b - lineLength * a);
        DrawLine(img, w, h, x0, y0, x1, y1, 0, 0, 255);
    }
}

void DrawSelectedCircles(int* H, int w, int h, Byte* img, int radius, int count) {
    struct C { int x, y, v; } best[100];
    for (int i = 0; i < count; i++) best[i] = { 0, 0, 0 };
    for (int y = 0; y < h; y++) for (int x = 0; x < w; x++) {
        int v = H[y * w + x];
        for (int i = 0; i < count; i++) {
            if (v > best[i].v) {
                for (int j = count - 1; j > i; j--) best[j] = best[j - 1];
                best[i] = { x, y, v };
                break;
            }
        }
    }
    for (int i = 0; i < count; i++)
        if (best[i].x - radius >= 0 && best[i].x + radius < w && best[i].y - radius >= 0 && best[i].y + radius < h)
            DrawCircle(img, w, h, best[i].x, best[i].y, radius, 0, 255, 0);
}

int main() {
    cv::Mat image = cv::imread("input.jpg", cv::IMREAD_COLOR);
    if (image.empty()) {
        std::cout << "Resim yuklenemedi!" << std::endl;
        return -1;
    }

    int w = image.cols, h = image.rows;
    Byte* raw = new Byte[w * h];

    // RGB'den gri tonlamaya manuel dönüþüm
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            cv::Vec3b pixel = image.at<cv::Vec3b>(y, x);
            Byte b = pixel[0];
            Byte g = pixel[1];
            Byte r = pixel[2];
            raw[y * w + x] = Byte(0.114 * b + 0.587 * g + 0.299 * r);
        }
    }

    Byte* smooth = Smoothing(raw, w, h);
    double* angles = nullptr;
    int* grad = Gradient(smooth, w, h, angles);
    Byte* edge = CannyEdge(grad, w, h, angles);
    Byte* color = CreateColorBuffer(edge, w, h);

    int hw, hh;
    int* hough_lines = Hough_Line(edge, w, h, hw, hh);
    int* hough_circles = Hough_Circle(edge, w, h, 20, hw, hh);

    DrawSelectedLines(hough_lines, hw, hh, color, w, h, 5);
    DrawSelectedCircles(hough_circles, w, h, color, 20, 3);

    cv::Mat output(h, w, CV_8UC3, color);
    cv::imshow("Detected Edges, Lines and Circles", output);
    cv::waitKey(0);

    // Bellek temizliði
    delete[] raw;
    delete[] smooth;
    delete[] grad;
    delete[] angles;
    delete[] edge;
    delete[] color;
    delete[] hough_lines;
    delete[] hough_circles;

    return 0;
}

