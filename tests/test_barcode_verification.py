import unittest
from segd.barcodes import label_image, verify_barcode_crop


class BarcodeVerificationTests(unittest.TestCase):
    def test_redecode_checks_pixels_and_syntax(self):
        image = label_image("W0120")
        self.assertTrue(verify_barcode_crop(image, "W0120"))
        self.assertFalse(verify_barcode_crop(image, "W0520"))
        self.assertFalse(verify_barcode_crop(image, 'W,"!'))
