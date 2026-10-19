# Tích hợp RNG9 SINE9 (v2.8)

Xem `../../BAT_DAU_RNG9.md` để chạy hai chế độ ADC RAW và RNG 9-bit với tab RNG riêng. Firmware bắt buộc có `RNG9=1` trong GET_INFO.

# STM32 ADC Data Logger

Tên cố định; phiên bản **2.7.1** hiển thị riêng. App thu PA0 qua USB CDC hoặc USB–TTL UART, đồ thị và CSV; tự kiểm tra và lưu cấu hình phù hợp board/PC.

1. Nếu đã nạp firmware 2.7 thì chỉ cần thay app, không nạp lại mạch. Nếu chưa có firmware, dùng toàn bộ project 2.7 và Keil Rebuild All/Download/Reset/Run.
2. Chạy RUN.bat tại đây hoặc MO_APP.bat ở gốc; Python 3.10+, Tcl/Tk, pip/venv. pySerial cài offline từ vendor. Không copy .venv/EXE cũ.
3. Kết nối đúng COM. App tự hỏi GET_INFO, chỉ bật **Tự kiểm tra & chọn tốc độ** khi thấy ADC_BIN_2_7.
4. Nhấn nút tự kiểm tra phía trên đồ thị một lần, đợi báo cáo. App tự đo các mức, kiểm lượt dài, hạ mức khi lỗi, chọn mức có dự phòng rồi lưu.
5. Sau đó chỉ cần START với cấu hình được chọn. Hồ sơ được dùng lại khi USB serial/COM và firmware khớp. STOP hủy bài đang chạy; không tự thu lại sau hủy.

Mặc định chưa có hồ sơ: **50000 Hz/100000 mẫu**, thu danh định 2 s. Trần thiết kế **100000 Hz/1000000 mẫu**; lượt trên 512 phải có N chia hết cho 250 để giữ DMA nửa khối 250 cố định. Mức cao nhất thực tế nằm trong qualification.json, khác với trần thiết kế. Nếu qua 100 kHz/1 triệu mẫu, app kiểm thêm 80 kHz/1 triệu mẫu hai lượt để chọn mức vận hành.

Bài đầy đủ có khoảng 45 s thu dữ liệu khi qua mọi mức, cộng truyền/lưu; nếu chỉ đạt mức thấp hơn có thể lâu hơn. Mỗi bước cần đủ mẫu, session/thứ tự/CRC, thời gian hợp lý, FIFO <=75% và số đo copy/đóng gói nếu DWT có sẵn. Mô phỏng không được ghi làm hồ sơ phần cứng.

Các lượt kiểm tra trung gian giữ raw byte và tóm tắt CRC/metrics; chỉ lượt vận hành cuối lưu CSV đầy đủ. Thu bình thường luôn lưu CSV sau PASS. qualification.json chứa lịch và kết quả, captures.csv có cột thời gian MCU/FIFO/CPU, rx_raw.bin giữ dữ liệu nhị phân đầy đủ. Nút Xuất CSV ADC ưu tiên lượt đầy đủ gần nhất. Đồ thị chỉ giữ 2500 điểm cuối, không lấy thưa dữ liệu CRC/CSV.

App nhận block 4096 byte trên thread riêng, xử lý tối đa 21 mẫu/thông báo. GUI cập nhật tiến độ khoảng 10 Hz, đồ thị 5 Hz. CONFIG/START/STOP/MONITOR dạng chữ; dữ liệu binary 62 byte và MTR metrics. Có thể đọc dữ liệu nhị phân 2.4/2.5 và ASCII 2.3, nhưng firmware cũ không chạy bài kiểm tra mới.

Tên dùng APP_NAME trong trng/__init__.py, không đổi theo version. Cấu trúc thư mục và chỗ lưu cũ giữ để tương thích. SIZE chỉ dự phòng, chưa sinh random; simulator là dữ liệu tổng hợp. Gói cung cấp mã nguồn Python, chưa có EXE Windows mới.

Kiểm tra: `python -m unittest discover -s tests -p "test_*.py" -v`. Host tests không đo được tốc độ board, độ chính xác analog hoặc hiệu năng GUI Windows. Xem PROJECT_DESIGN.md và DESIGN_BUDGET.json ở gốc để xem tính toán.

## Dùng UART qua USB–TTL

Chọn **UART (USB–TTL)** ở mục Kết nối thiết bị; chọn COM của adapter rồi Kết nối. App đặt 115200 baud, mặc định 1000 Hz / 10000 mẫu, giới hạn 3000 Hz. PA9→RX adapter, PA10←TX adapter, GND↔GND; tín hiệu TTL 3,3 V.

Dùng các nút Gửi cấu hình, START, STOP, ADC Monitor, đồ thị và CSV trên cùng giao diện như CDC. App 2.7.1 không có tab/nút thử nối tắt PA9–PA10. Chờ INFO xác nhận cổng trước khi START; baud 115200 trên CDC không giới hạn tốc độ USB. Nếu nhận LINK=USB dù chọn UART, app chuyển hiển thị về CDC và nhắc chọn đúng COM của USB–TTL.

Muốn dùng USB trực tiếp, chọn **USB CDC** rồi COM CDC của STM32. App áp mặc định 50000 Hz / 100000 mẫu khi chưa có hồ sơ; trần cấu hình 100000 Hz. UART chỉ hiển thị các mức lấy mẫu nhỏ hơn; bài tự chọn 10–100 kHz dành cho CDC. Đồ thị, kiểm CRC, lưu CSV và thao tác START/STOP dùng chung.

Gói app-only giải nén vào thư mục mới, chạy MO_APP.bat. Cần Python 3.10+ có Tkinter và pip/venv; pySerial cài offline từ vendor. Đây là mã nguồn Python, chưa có EXE Windows. Tên app giữ nguyên STM32 ADC Data Logger.
