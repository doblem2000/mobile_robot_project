import numpy as np


class FromPatchToDet():
    def __init__(self):
        pass

    # Funzione per calcolare la proporzione normalizzata
    def normalize_bbox(self, bbox, image_shapes):
        return np.array([x / y for x, y in zip(bbox, image_shapes)], dtype=np.float32)

    # Esempio di poligoni, quadrilateri e quadrilateri con padding
    def generate_polygon(self, bbox):
        x1, y1, x2, y2 = bbox
        return np.array([
            [x1, y1], [x1, y2], [x2, y2], [x2, y1],
            [x1 + 5, y1 + 5], [x2 - 5, y1 + 5], [x2 - 5, y2 - 5], [x1 + 5, y2 - 5]
        ], dtype=np.float32)

    def generate_quad(self, bbox):
        x1, y1, x2, y2 = bbox
        return np.array([
            [x1, y1], [x2, y1], [x2, y2], [x1, y2]
        ], dtype=np.float32)

    def pad_quad(self, quad, padding=5):
        return np.array([
            [quad[0][0] - padding, quad[0][1] - padding],
            [quad[1][0] + padding, quad[1][1] - padding],
            [quad[2][0] + padding, quad[2][1] + padding],
            [quad[3][0] - padding, quad[3][1] + padding]
        ], dtype=np.float32)


    def convert_result(self, img, result):
        # Supponiamo che 'img' sia la tua immagine e 'result' contenga i risultati del rilevamento
        height, width = img.shape[:2]
        image_shapes = [width, height, width, height]


        detections = []
        for confidence, bbox_xyxy in zip(result.filtered_confidences, result.filtered_boxes):
            bbox_xyxy = np.array(bbox_xyxy, dtype=np.float32)
            bbox_xyxyn = self.normalize_bbox(bbox_xyxy, image_shapes)
            cxcy = ((bbox_xyxy[0] + bbox_xyxy[2]) / 2, (bbox_xyxy[1] + bbox_xyxy[3]) / 2)
            cxcyn = (cxcy[0] / width, cxcy[1] / height)
            wh = (bbox_xyxy[2] - bbox_xyxy[0], bbox_xyxy[3] - bbox_xyxy[1])
            whn = (wh[0] / width, wh[1] / height)
            polygon_xy = self.generate_polygon(bbox_xyxy)
            polygon_xyn = self.normalize_bbox(polygon_xy.flatten(), [width, height] * (polygon_xy.size // 2)).reshape(polygon_xy.shape)
            quad_xy = self.generate_quad(bbox_xyxy)
            quad_xyn = self.normalize_bbox(quad_xy.flatten(), [width, height] * (quad_xy.size // 2)).reshape(quad_xy.shape)
            padded_quad_xy = self.pad_quad(quad_xy)
            padded_quad_xyn = self.normalize_bbox(padded_quad_xy.flatten(), [width, height] * (padded_quad_xy.size // 2)).reshape(padded_quad_xy.shape)

            d = {
                'confidence': confidence,
                'bbox_xyxy': bbox_xyxy,
                'bbox_xyxyn': bbox_xyxyn,
                'cxcy': cxcy,
                'cxcyn': cxcyn,
                'wh': wh,
                'whn': whn,
                'polygon_xy': polygon_xy,
                'polygon_xyn': polygon_xyn,
                'quad_xy': quad_xy,
                'quad_xyn': quad_xyn,
                'padded_quad_xy': padded_quad_xy,
                'padded_quad_xyn': padded_quad_xyn,
                'image_shape': (height, width)
            }
            detections.append(d)
        return detections