import cv2

from autonomous_navigation_challenge_2024.pipelines.pipeline import Pipeline

class CV2SWPipeline(Pipeline):
    def __init__(self, SHOW=False):
        self.qr_detector = cv2.QRCodeDetector()
        self.SHOW = SHOW
        
    def display_detections(self, cv_image, detections):
        for info, bb in detections:
            pt1 = (int(bb[0][0]), int(bb[0][1]))
            pt2 = (int(bb[1][0]), int(bb[1][1]))
            pt3 = (int(bb[2][0]), int(bb[2][1]))
            pt4 = (int(bb[3][0]), int(bb[3][1]))
            cv_image = cv2.line(cv_image, pt1, pt2, (0, 0, 255), 10)
            cv_image = cv2.line(cv_image, pt2, pt3, (0, 0, 255), 10)
            cv_image = cv2.line(cv_image, pt3, pt4, (0, 0, 255), 10)
            cv_image = cv2.line(cv_image, pt4, pt1, (0, 0, 255), 10)
            cv_image = cv2.putText(cv_image, info, pt1, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        return cv_image
    
    def process_image(self, cv_image, draw_results = False):
        height, width = cv_image.shape[:2]  # Get the height and width of the image
        window_size = (width // 3, height // 3)  # Width, Height of the window
        width_step_size = window_size[1]  # Step size should match the window width
        height_step_size = window_size[0]

        # Calculate area boundaries
        area_height = height // 3
        area_width = width // 3

        # Draw red grid for 3x3 areas
        if self.SHOW:
            for i in range(1, 3):
                cv2.line(cv_image, (0, i * area_height), (width, i * area_height), (0, 0, 255), 2)
            for j in range(1, 3):
                cv2.line(cv_image, (j * area_width, 0), (j * area_width, height), (0, 0, 255), 2)

        # cv2.imshow("Camera", cv_image)
        # cv2.waitKey(1)

        for y in range(0, height - window_size[1] + 1, width_step_size):
            for x in range(0, width - window_size[0] + 1, height_step_size):
                window = cv_image[y:y + window_size[1], x:x + window_size[0]]
                window_center_x = x + window_size[0] // 2  # Center x of the sliding window
                window_center_y = y + window_size[1] // 2  # Center y of the sliding window

                if window.shape[0] != window_size[1] or window.shape[1] != window_size[0]:
                    continue

                # clone = cv_image.copy()

                # if self.SHOW:
                #     cv2.rectangle(clone, (x, y), (x + window_size[0], y + window_size[1]), (0, 255, 0), 2)
                #     cv2.imshow("Camera", clone)
                #     cv2.waitKey(1)

                retval, decoded_info, points, straight_qrcode = self.qr_detector.detectAndDecodeMulti(cv_image)
                if retval:
                                   
                    # Determine the area in which the center of the window is located
                    area_row = window_center_y // area_height
                    area_col = window_center_x // area_width
                    area_number = area_row * 3 + area_col + 1
                    if draw_results:
                        cv_image = self.display_detections(cv_image, zip(decoded_info, points))

        return False, cv_image, None