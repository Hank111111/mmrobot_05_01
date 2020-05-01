import pyqrcode
# see https://pythonhosted.org/PyQRCode/create.html
import img2pdf
# see https://gitlab.mister-muffin.de/josch/img2pdf
import numpy as np
import io
import cv2
MM_PER_INCH = 25.4 
PAPER_SIZE_MM = np.array([210, 297]) # A4 , see https://www.papersizes.org/a-paper-sizes.htm
PRINT_DPI = 300
PRINT_ROWS = 4
PRINT_COLS = 3

PAPER_SIZE_PIXEL = (PAPER_SIZE_MM / MM_PER_INCH * PRINT_DPI).astype(np.int)
CELL_WIDTH = int(PAPER_SIZE_PIXEL[0] / PRINT_COLS)
CELL_HEIGHT = int(PAPER_SIZE_PIXEL[1] / PRINT_ROWS)

EACH_QR_CODE_MIN_PADDING_MM = 10
QR_CODE_SIZE_MM = 50
EACH_QR_CODE_MIN_PADDING_PIXEL = int(EACH_QR_CODE_MIN_PADDING_MM / MM_PER_INCH * PRINT_DPI)
QR_CODE_SIZE_PIXEL = int(QR_CODE_SIZE_MM / MM_PER_INCH * PRINT_DPI)
assert(CELL_HEIGHT > QR_CODE_SIZE_MM + EACH_QR_CODE_MIN_PADDING_MM * 2)
assert(CELL_WIDTH > QR_CODE_SIZE_MM + EACH_QR_CODE_MIN_PADDING_MM * 2)

def generateQRCodeImage(content):
    qrcode = pyqrcode.create(content)
    buffer = io.BytesIO()
    qrcode.png(buffer, 40, quiet_zone=0)
    file_bytes = np.asarray(bytearray(buffer.getvalue()), dtype=np.uint8)
    qr_img = cv2.imdecode(file_bytes, cv2.IMREAD_COLOR)
    #cv2.imshow("image", img)
    #cv2.waitKey(0)
    return qr_img
def generateCell(content):
    cell_img = np.ones((CELL_HEIGHT, CELL_WIDTH, 3), dtype=np.uint8) * 255
    #put the text in the center of row

    qr_img = generateQRCodeImage(content)
    qr_img = cv2.resize(qr_img, (QR_CODE_SIZE_PIXEL, QR_CODE_SIZE_PIXEL)) 

    # setup text
    font = cv2.FONT_HERSHEY_COMPLEX
    # get boundary of this text
    text_width, text_height = cv2.getTextSize(content, font, 2, 3)[0]

    padding_width = int((CELL_WIDTH - QR_CODE_SIZE_PIXEL)/2)
    padding_height = int((CELL_HEIGHT - QR_CODE_SIZE_PIXEL - EACH_QR_CODE_MIN_PADDING_PIXEL - text_height)/2)
    
    cell_img[padding_height:(padding_height+QR_CODE_SIZE_PIXEL), padding_width:(padding_width+QR_CODE_SIZE_PIXEL)] = qr_img
    

    # get coords based on boundary
    text_center_x = CELL_WIDTH/2
    text_center_y = padding_height + QR_CODE_SIZE_PIXEL + EACH_QR_CODE_MIN_PADDING_PIXEL + text_height/2
    textX = int(round(text_center_x - text_width / 2))
    print(text_center_x, text_width)
    textY = int(round(text_center_y + text_height/2))

    # add text centered on image
    cv2.putText(cell_img, content, (textX, textY ), font, 2, (0, 0, 0), 3)
    cv2.imshow("cell_img", cell_img)
    cv2.waitKey(0)

if __name__ == "__main__":
    generateCell("0-0-0")

