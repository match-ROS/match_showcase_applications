import csv
import os
from PyQt5.QtWidgets import QTableWidgetItem

CSV_FILE = "relative_poses.csv"

def save_relative_poses(table):
    """Saves the current values from the table to a CSV file."""
    with open(CSV_FILE, "w", newline="") as file:
        writer = csv.writer(file)
        for row in range(8):
            writer.writerow([table.item(row, col).text() if table.item(row, col) else "0.0" for col in range(3)])

def load_relative_poses(table):
    """Loads relative poses from the CSV file if it exists."""
    if os.path.exists(CSV_FILE):
        with open(CSV_FILE, "r") as file:
            reader = csv.reader(file)
            for row_idx, row in enumerate(reader):
                for col_idx, value in enumerate(row):
                    table.setItem(row_idx, col_idx, QTableWidgetItem(value))
    else:
        for row in range(8):
            for col in range(3):
                table.setItem(row, col, QTableWidgetItem("0.0"))
