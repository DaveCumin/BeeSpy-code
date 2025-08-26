import sys
import numpy as np
import pandas as pd
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
                             QWidget, QPushButton, QLabel, QSlider, QSpinBox, 
                             QFileDialog, QMessageBox)
from PyQt5.QtCore import Qt, pyqtSignal
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.colors import ListedColormap

class SpectrogramCanvas(FigureCanvas):
    """Custom matplotlib canvas for PyQt5"""
    def __init__(self, parent=None, width=12, height=8, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        super(SpectrogramCanvas, self).__init__(self.fig)
        self.setParent(parent)
        
        # Create subplots
        self.ax1 = self.fig.add_subplot(311)
        self.ax2 = self.fig.add_subplot(312)
        self.ax3 = self.fig.add_subplot(313)
        
        # Configure subplots
        self.ax1.set_title('Original Data')
        self.ax1.set_xlabel('Time')
        self.ax1.set_ylabel('Frequency')
        
        self.ax2.set_title('Transformed Data (Above Threshold)')
        self.ax2.set_xlabel('Time')
        self.ax2.set_ylabel('Frequency')
        
        self.ax3.set_title('Power Sum Comparison (Within Frequency Range)')
        self.ax3.set_xlabel('Time')
        self.ax3.set_ylabel('Power Sum')
        
        self.fig.tight_layout()
        
        # Store colorbars and plot lines
        self.cbar1 = None
        self.cbar2 = None

class SpectrogramViewer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Spectrogram Viewer")
        self.setGeometry(100, 100, 1200, 800)
        
        # Data storage
        self.times = None
        self.freqs = None
        self.values = None
        self.transformed_data = None
        self.cutoff = 80.0
        self.min_freq = 0.0
        self.max_freq = 100.0
        
        # Setup GUI
        self.setup_gui()

    
    def setup_gui(self):
        """Setup the GUI layout"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        layout = QVBoxLayout(central_widget)
        
        # File selection
        file_layout = QHBoxLayout()
        self.load_button = QPushButton("Load CSV File")
        self.load_button.clicked.connect(self.load_file)
        file_layout.addWidget(self.load_button)
        
        self.file_label = QLabel("No file loaded")
        file_layout.addWidget(self.file_label)
        file_layout.addStretch()
        
        layout.addLayout(file_layout)
        
        # Controls
        controls_layout = QHBoxLayout()
        controls_layout.addWidget(QLabel("Cutoff Percentile:"))
        
        # Slider
        self.cutoff_slider = QSlider(Qt.Horizontal)
        self.cutoff_slider.setMinimum(0)
        self.cutoff_slider.setMaximum(100)
        self.cutoff_slider.setValue(80)
        self.cutoff_slider.valueChanged.connect(self.on_cutoff_change)
        controls_layout.addWidget(self.cutoff_slider)
        
        # Spin box
        self.cutoff_spinbox = QSpinBox()
        self.cutoff_spinbox.setMinimum(0)
        self.cutoff_spinbox.setMaximum(100)
        self.cutoff_spinbox.setValue(80)
        self.cutoff_spinbox.valueChanged.connect(self.on_spinbox_change)
        controls_layout.addWidget(self.cutoff_spinbox)
        
        # Threshold display
        self.threshold_label = QLabel("Threshold: 0.0")
        controls_layout.addWidget(self.threshold_label)
        controls_layout.addStretch()
        
        layout.addLayout(controls_layout)
        
        # Frequency range controls
        freq_layout = QHBoxLayout()
        freq_layout.addWidget(QLabel("Frequency Range:"))
        
        freq_layout.addWidget(QLabel("Min:"))
        self.min_freq_spinbox = QSpinBox()
        self.min_freq_spinbox.setMinimum(0)
        self.min_freq_spinbox.setMaximum(10000)
        self.min_freq_spinbox.setValue(0)
        self.min_freq_spinbox.valueChanged.connect(self.on_freq_range_change)
        freq_layout.addWidget(self.min_freq_spinbox)
        
        freq_layout.addWidget(QLabel("Max:"))
        self.max_freq_spinbox = QSpinBox()
        self.max_freq_spinbox.setMinimum(0)
        self.max_freq_spinbox.setMaximum(10000)
        self.max_freq_spinbox.setValue(100)
        self.max_freq_spinbox.valueChanged.connect(self.on_freq_range_change)
        freq_layout.addWidget(self.max_freq_spinbox)
        
        freq_layout.addStretch()
        
        layout.addLayout(freq_layout)
        
        # Plot canvas
        self.canvas = SpectrogramCanvas(self)
        layout.addWidget(self.canvas)
    
    def load_file(self):
        """Load and parse CSV file"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, 
            "Select CSV file", 
            "", 
            "CSV files (*.csv);;All files (*.*)"
        )
        
        if not file_path:
            return
        
        try:
            # Load CSV with pandas for robust parsing
            df = pd.read_csv(file_path, index_col=0)
            
            # Extract data
            self.times = df.columns.astype(float).values
            self.freqs = df.index.astype(float).values
            self.values = df.values
            
            # Update file label
            import os
            self.file_label.setText(f"Loaded: {os.path.basename(file_path)}")
            
            # Update frequency range controls based on actual data
            if len(self.freqs) > 0:
                self.min_freq_spinbox.setMinimum(int(self.freqs.min()))
                self.min_freq_spinbox.setMaximum(int(self.freqs.max()))
                self.min_freq_spinbox.setValue(int(self.freqs.min()))
                
                self.max_freq_spinbox.setMinimum(int(self.freqs.min()))
                self.max_freq_spinbox.setMaximum(int(self.freqs.max()))
                self.max_freq_spinbox.setValue(int(self.freqs.max()))
                
                self.min_freq = self.freqs.min()
                self.max_freq = self.freqs.max()
            
            # Draw plots
            self.draw_original_plot()
            self.update_transformed_data()
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load file:\n{str(e)}")
    
    def get_quantile_threshold(self, cutoff_percent):
        """Calculate the quantile threshold"""
        if self.values is None:
            return 0.0
        
        flat_values = self.values.flatten()
        ##remove nans
        flat_values = flat_values[~np.isnan(flat_values)]
        return np.percentile(flat_values, cutoff_percent)
    
    def on_cutoff_change(self, value):
        """Handle slider changes"""
        self.cutoff = value
        self.cutoff_spinbox.setValue(value)
        self.update_transformed_data()
    
    def on_freq_range_change(self):
        """Handle frequency range changes"""
        self.min_freq = self.min_freq_spinbox.value()
        self.max_freq = self.max_freq_spinbox.value()
        
        # Ensure min <= max
        if self.min_freq > self.max_freq:
            if self.sender() == self.min_freq_spinbox:
                self.max_freq_spinbox.setValue(self.min_freq)
                self.max_freq = self.min_freq
            else:
                self.min_freq_spinbox.setValue(self.max_freq)
                self.min_freq = self.max_freq
        
        self.draw_power_comparison()
    
    def on_spinbox_change(self, value):
        """Handle spinbox changes"""
        self.cutoff = value
        self.cutoff_slider.setValue(value)
        self.update_transformed_data()
    
    def on_freq_range_change(self):
        """Handle frequency range changes"""
        self.min_freq = self.min_freq_spinbox.value()
        self.max_freq = self.max_freq_spinbox.value()
        
        # Ensure min <= max
        if self.min_freq > self.max_freq:
            if self.sender() == self.min_freq_spinbox:
                self.max_freq_spinbox.setValue(self.min_freq)
                self.max_freq = self.min_freq
            else:
                self.min_freq_spinbox.setValue(self.max_freq)
                self.min_freq = self.max_freq
        
        self.draw_power_comparison()
    
    def get_frequency_mask(self):
        """Get boolean mask for frequencies within the specified range"""
        if self.freqs is None:
            return None
        return (self.freqs >= self.min_freq) & (self.freqs <= self.max_freq)
    
    def calculate_power_sums(self):
        """Calculate power sums within frequency range for original and transformed data"""
        if self.values is None or self.transformed_data is None:
            return None, None
        
        freq_mask = self.get_frequency_mask()
        if freq_mask is None or not freq_mask.any():
            return None, None
        
        # Sum power within frequency range for each time point
        original_power = np.sum(self.values[freq_mask, :], axis=0)
        transformed_power = np.sum(self.transformed_data[freq_mask, :], axis=0)
        
        return original_power, transformed_power
    
    def update_transformed_data(self):
        """Update transformed data and redraw"""
        if self.values is None:
            return
        
        threshold = self.get_quantile_threshold(self.cutoff)
        
        # Update threshold display
        self.threshold_label.setText(f"Threshold: {threshold:.3f}")
        
        # Create transformed data (set values below threshold to 0)
        self.transformed_data = np.where(self.values > threshold, self.values, 0)
        
        # Redraw transformed plot
        self.draw_transformed_plot()
        
        # Draw power comparison plot
        self.draw_power_comparison()
    
    def draw_original_plot(self):
        """Draw the original data plot"""
        if self.values is None:
            return
        
        # Remove existing colorbar safely
        if self.canvas.cbar1 is not None:
            try:
                self.canvas.cbar1.remove()
            except:
                pass
            self.canvas.cbar1 = None
        
        # Clear and redraw original plot
        self.canvas.ax1.clear()
        self.canvas.ax1.set_title('Original Data')
        self.canvas.ax1.set_xlabel('Time')
        self.canvas.ax1.set_ylabel('Frequency')
        
        im1 = self.canvas.ax1.imshow(
            self.values,
            aspect='auto',
            origin='lower',
            extent=[self.times[0], self.times[-1], self.freqs[0], self.freqs[-1]],
            cmap='viridis',
            interpolation='nearest'
        )
        
        # Set consistent x-axis limits
        self.canvas.ax1.set_xlim(self.times[0], self.times[-1])
        
        # Add colorbar
        #self.canvas.cbar1 = self.canvas.fig.colorbar(im1, ax=self.canvas.ax1, shrink=0.8)
        
        self.canvas.draw()
    
    def draw_transformed_plot(self):
        """Draw the transformed data plot"""
        if self.transformed_data is None:
            return
        
        # Remove existing colorbar safely
        if self.canvas.cbar2 is not None:
            try:
                self.canvas.cbar2.remove()
            except:
                pass
            self.canvas.cbar2 = None
        
        # Clear and redraw transformed plot
        self.canvas.ax2.clear()
        self.canvas.ax2.set_title('Transformed Data (Above Threshold)')
        self.canvas.ax2.set_xlabel('Time')
        self.canvas.ax2.set_ylabel('Frequency')
        
        im2 = self.canvas.ax2.imshow(
            self.transformed_data,
            aspect='auto',
            origin='lower',
            extent=[self.times[0], self.times[-1], self.freqs[0], self.freqs[-1]],
            cmap='viridis',
            interpolation='nearest'
        )
        
        # Set consistent x-axis limits
        self.canvas.ax2.set_xlim(self.times[0], self.times[-1])
        
        # Add colorbar
        #self.canvas.cbar2 = self.canvas.fig.colorbar(im2, ax=self.canvas.ax2, shrink=0.8)
        
        self.canvas.draw()
    
    def draw_power_comparison(self):
        """Draw the power comparison plot"""
        original_power, transformed_power = self.calculate_power_sums()
        
        if original_power is None or transformed_power is None:
            return
        
        # Completely remove and recreate the third subplot to clear twinx
        self.canvas.ax3.remove()
        self.canvas.ax3 = self.canvas.fig.add_subplot(313)
        
        self.canvas.ax3.set_title(f'Power Sum Comparison (Freq: {self.min_freq:.1f}-{self.max_freq:.1f})')
        self.canvas.ax3.set_xlabel('Time')
        
        # Plot original data on left y-axis
        # Remove existing ticks
        self.canvas.ax3.set_yticks([])
        line1 = self.canvas.ax3.plot(self.times, original_power, 'b-', linewidth=2, label='Original Data', alpha=0.7)
        self.canvas.ax3.set_ylabel('Original Power Sum', color='b')
        self.canvas.ax3.tick_params(axis='y', labelcolor='b')
        
        # Create second y-axis for transformed data
        ax3_twin = self.canvas.ax3.twinx()
        # Remove existing ticks
        ax3_twin.set_yticks([])
        line2 = ax3_twin.plot(self.times, transformed_power, 'r-', linewidth=2, label='Transformed Data', alpha=0.7)
        ax3_twin.set_ylabel('Transformed Power Sum', color='r')
        ax3_twin.tick_params(axis='y', labelcolor='r')
        
        # Force exact same x-axis limits for all plots
        x_min, x_max = self.times[0], self.times[-1]
        self.canvas.ax3.set_xlim(x_min, x_max)
        ax3_twin.set_xlim(x_min, x_max)
        
        # Also ensure spectrograms have same limits
        self.canvas.ax1.set_xlim(x_min, x_max)
        self.canvas.ax2.set_xlim(x_min, x_max)
        
        # Add grid only to main axis
        self.canvas.ax3.grid(True, alpha=0.3)
        
        # Adjust layout and redraw
        self.canvas.fig.tight_layout()
        self.canvas.draw()

def main():
    """Main function to run the application"""
    app = QApplication(sys.argv)
    window = SpectrogramViewer()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()