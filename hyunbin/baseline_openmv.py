import sensor, image, time
import time
from pyb import UART
'''
(벽 왼쪽 부터 반시계방향으로 A,B,C,D)
(벽 왼쪽 부터 반시계방향으로 A,B,C,D)
A{
rgb:(150,200,40,100,0,50)
distance:20
area:100
}
B:{
rgb:(160,210,60,190,0,50)
distance:20
area:100
}
C:{
rgb:(160,250,90,150,0,60)
distance:20
area:100
}
D:{
rgb:(160,250,60,140,0,60)
distance:20
area:100
}
'''

threshold_red = (160,250,90,150,0,60)
MIN_AREA_THRESHOLD = 100  # 필터링 임계값 설정
DISTANCE_THRESHOLD = 20   # 거리 임계값 설정

uart = UART(3,115200,timeout_char=200)

# 1단계 코드: 카메라 스트리밍
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)  # 해상도를 QQVGA로 낮춤 (160x120)
sensor.skip_frames(time=2000)
clock = time.clock()

# 2단계 코드: Blob 만들기
class Blob:
    def __init__(self, x, y, w, h):
        self.x = x
        self.y = y
        self.w = w
        self.h = h

    def rect(self):
        return (self.x, self.y, self.w, self.h)

    def area(self):
        return self.w * self.h  # 사각형 면적 계산

# 3단계 코드: 특정 색상 범위로 블롭 감지

def find_blobs(img, threshold):
    width = img.width()
    height = img.height()
    blobs = []

    for y in range(0, height, 5):
        for x in range(0, width, 5):
            pixel = img.get_pixel(x, y)
            r, g, b = pixel
            if threshold[0] <= r <= threshold[1] and threshold[2] <= g <= threshold[3] and threshold[4] <= b <= threshold[5]:
                blobs.append(Blob(x, y, 5, 5))

    return blobs

# 4단계 코드: 블롭 병합
def merge_blobs(blobs, distance_threshold=50):
    merged_blobs = []

    while blobs:
        current_blob = blobs.pop(0)
        merged = False

        for merged_blob in merged_blobs:
            dx = abs(current_blob.x - merged_blob.x)
            dy = abs(current_blob.y - merged_blob.y)

            if dx <= distance_threshold and dy <= distance_threshold:
                x1 = min(current_blob.x, merged_blob.x)
                y1 = min(current_blob.y, merged_blob.y)
                x2 = max(current_blob.x + current_blob.w, merged_blob.x + merged_blob.w)
                y2 = max(current_blob.y + current_blob.h, merged_blob.y + merged_blob.h)

                merged_blob.x = x1
                merged_blob.y = y1
                merged_blob.w = x2 - x1
                merged_blob.h = y2 - y1

                merged = True
                break

        if not merged:
            merged_blobs.append(current_blob)

    return merged_blobs

# 5단계 코드: 블롭 크기 필터링

while(True):
    clock.tick()
    img = sensor.snapshot()
    blobs = find_blobs(img, threshold_red)
    time.sleep(0.05)
    merged_blobs = merge_blobs(blobs,DISTANCE_THRESHOLD)

    for blob in merged_blobs:
        if blob.area() >= MIN_AREA_THRESHOLD:
            img.draw_rectangle(blob.rect())
            img.draw_cross(blob.x + blob.w // 2, blob.y + blob.h // 2)
        center_x = blob.x + blob.w//2
        center_y = blob.y + blob.h//2
        msg = [center_x, center_y]
        data_to_send = str(msg)  # 리스트를 문자열로 변환
        uart.write(data_to_send)  # 문자열을 UART로 전송
        # 블롭 정보 출력
        print("Blob [면적: %d, 중심 좌표: (%d, %d), 가로: %d, 세로: %d]" %
              (blob.area(), blob.x + blob.w//2, blob.y + blob.h//2, blob.w, blob.h))
