# GUI RobStride 7 motor - V29

Cap nhat ngay 15/10/2026. Chay truc tiep tren Windows bang Python, Tkinter,
pyserial va bo chuyen USB-CAN AT hien tai. Van dung ten `gui_3motor_pc.py`.

## Cau hinh da xac nhan

| CAN ID | Dong co |
| --- | --- |
| 1 | EL05 |
| 2 | RS00 |
| 3 | RS00 |
| 4 | EL05 |
| 5 | EL05 |
| 6 | EL05 |
| 7 | RS00 |

`motor_config_gui.json` va cau hinh mac dinh trong GUI cung dung bang nay.
Chuong trinh khong doi CAN ID tren motor. M1 nay la EL05, khac cau hinh RS00
cu: can day lai waypoint. Moi goc trong GUI la goc truc ra motor, don vi do/rad;
khong tu quy doi sang hanh trinh ray hoac goc khop theo ti so truyen ngoai.

## Mo giao dien de review

```bat
cd /d D:\TailieuKLTN\KLTN_robot
python gui_3motor_pc.py
```

Tieu de: `RobStride 7-Motor Local Controller - V29 Command Guard`.
Mo GUI chua mo COM va chua gui CAN. Python can co Tkinter va pyserial nhu ban cu.
Neu VS Code dung interpreter khac, chon lai interpreter co hai thanh phan nay.

Co the chay `python pc_motor_controllerpy.py` de mo cung GUI. File nay khong
con la cong cu quet/doi ID cu. Kiem tra cau hinh ma khong mo cong:

```bat
python pc_motor_controllerpy.py --check-config
```

`--list-ports` chi liet ke cong serial. Chay rieng `robstride_driver_v0.py`
chi in huong dan, khong tu chay demo motor.

## Nhung phan da nang cap

- Hien thi 7 motor theo dung ID/profile: goc do/rad, van toc rad/s, Iq A,
  `M~` Nm. `M~` la uoc tinh `Iq * Kt` tu profile, khong phai cam bien luc doc lap.
  So lieu chua nhan hoac da cu hien `--`; goc cu/mat ket noi duoc danh dau.
- Mot serial handle va khoa chung cho ca 7 driver. Chi tao ket noi khi bam
  Ket noi. Thong bao mo COM khong dong nghia ca 7 motor da tra feedback.
- Bat luc xac nhan PP mode, doc vi tri moi, ghi/xac nhan HOLD khi disabled,
  roi moi ENABLE/Run. That bai o mot truc se thu STOP ca nhom va bao loi.
- STOP / Ngat All hoat dong ca trong Sequence, huy lenh manual dang cho va
  yeu cau lenh dang chay ket thuc; gui STOP cho moi motor truoc khi cho ACK.
- Teaching giu Kd=0.35, Kp=0 va torque feedforward=0 nhu ban da chinh. Kiem tra
  mode MIT va ENABLE cho tung motor; loi feedback Teaching yeu cau teardown.
  Dung Teaching thuong chuyen sang HOLD da xac nhan; vao Sequence thi STOP
  Teaching truoc de worker Sequence tu chuan bi PP.
- JOG buoc lay goc moi, xac nhan mode/HOLD/V-A/target; JOG giu xac nhan
  Speed mode/ENABLE. Tha chuot dung dung motor da bam, ke ca khi doi o chon.
  Tha truoc khi worker chay khong gui van toc khac 0. Mat focus app yeu cau dung JOG.
- Dat zero yeu cau STOP xac nhan cho ca nhom. Theo muc 4.2.6 cua hai manual
  260112, PP khong cho dat zero: GUI doc mode, chuyen tam sang Motion Control
  neu can, gui Type 6 va doc lai goc, roi khoi phuc mode cu khi van ngat luc.
  Chi gui lai Type 6 khi chua xac nhan, toi da 3 lan. Khong ENABLE trong qua trinh.
  Mot motor khong xac nhan zero van cho phep thu cac motor con lai va bao ro ID loi.
  Khong doc duoc mode thi khong doan mode de zero. Xoa waypoint trong bo nho.
  File waypoint da luu truoc khi doi moc zero can duoc day lai theo moc moi.
- Bang waypoint tu tao du 7 cot goc; co thanh cuon doc/ngang. Luu diem chi
  khi tat ca goc con moi. Nhan dup cot Cho(s) de sua thoi gian dung.
- Luu/Mo chuoi JSON: giu thu tu diem, goc va delay; kiem tra du ID/profile,
  du 7 goc, so huu han, delay khong am. Mo file khong tu chay motor.
- Sequence dung PP mode=1, xac nhan V/A truoc target va loc_ref sau target,
  doi tat ca motor den diem va dung on dinh, HOLD du delay roi moi sang diem tiep.
  RS00 xac nhan V/A bang readback; EL05 giu Type 2 ACK cho thanh ghi W-only.
- Nut Xem log hien 200 dong cuoi `sequence_diagnostics.log` de xem ly do loi.

## Sua loi lenh va do tre trong V29

- Telemetry nhuong RX sau request dang chay khi co lenh nut bam; khong doi
  tron mot vong doc 7 motor. Lenh manual dang xu ly hien trang thai ngay.
  Bam lap khong xep nhieu worker; STOP All va nha JOG van duoc uu tien.
- ENABLE All xac nhan STOP, chi ghi PP mode neu can, ghi/xac nhan HOLD
  tu goc moi roi moi ENABLE. HOLD bi mat duoc gui lai khi motor van disabled.
- Tren serial dung chung, cac frame hoan chinh cach nhau it nhat 6ms;
  PP target van giu 10ms nhu cu. Parser loai RX cu sau khoang cho, ngay truoc TX.
  Day la gioi han toc do gui, khong phai bao dam bus/adapter khong mat goi.
- Thieu ACK STOP khong tu dong dong COM: khoa lenh chuyen dong, van doc
  goc, yeu cau bam STOP All de xac nhan lai. STOP chua ACK khong dong nghia
  motor da dung. Loi serial/OS that van dung worker va dong ket noi nhu cu.
- Log co frame Type 6 va read timeout (gioi han 1 lan/giay/motor), de phan
  biet khong nhan byte, sai frame, loi mode va loi Windows serial.
- Dong GUI cu roi mo lai dung duong dan tren de nap code V29. Cua so Python
  da chay truoc khi sua file khong tu cap nhat code.

Nut Cau hinh 1-7 motor chi cho sua khi da ngat ket noi va worker da dung.
Thay danh sach/profile se yeu cau day lai waypoint. Khong sua file trong
`dist/Robot_Simulation`: app 3D do chua duoc ghep trong dot nang cap nay.

## Kiem thu va gioi han

180 kiem thu offline da dat, gom Tk an, 7 driver tren FakeSerial, parser AT,
Start/Stop, mat feedback, cac profile dung theo danh sach tren, Teaching/JOG,
luu/mo JSON va cac loi COM gia lap. Khong mo COM that hay gui CAN toi phan cung.

Nguon Windows/Python/pyserial va dinh dang frame giu nguyen. Gioi han motor,
thang giai ma RS00/EL05, baudrate 921600 va giao thuc khong doi. Moc goc,
ti so truyen ngoai va gioi han co khi can xac nhan voi robot that.

Chua co xac nhan thuc nghiem 7 motor. Moi target PP la mot frame AT rieng,
cach nhau danh nghia 10 ms: tu frame dau den frame thu 7 khoang 60 ms cong
tre Windows/adapter. Khong bao dam bat dau/ket thuc vat ly tuyet doi cung luc.
Khoang cach rat nho/rat lech nhau van chiu san V/A cu. Mat duong truyen co the
khien STOP khong toi motor; GUI phan biet STOP da/chua xac nhan.

File driver con cac helper bench/CSP cu, khong duoc GUI V29 dung de chay chuoi.
Khong tu chay bench. Dung lai de review source va cau hinh truoc khi thu motor that.
