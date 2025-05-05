#!/bin/bash

print_step() {
    echo -e "\n\033[1;34m===> $1\033[0m"
}

print_success() {
    echo -e "\033[1;32m✓ $1\033[0m"
}

print_error() {
    echo -e "\033[1;31m✗ $1\033[0m"
}

print_step "Setting up SenseBridge Environment..."

# Create virtual environment
if [ ! -d ".venv" ]; then
    print_step "Creating Python virtual environment..."
    python3 -m venv .venv
    if [ $? -ne 0 ]; then
        print_error "Failed to create virtual environment. Is python3-venv installed?"
        echo "Try: sudo apt-get install python3-venv"
        exit 1
    fi
    print_success "Virtual environment created"
else
    print_success "Virtual environment already exists"
fi

# Activate virtual environment
source .venv/bin/activate
if [ $? -ne 0 ]; then
    print_error "Failed to activate virtual environment"
    exit 1
fi
print_success "Virtual environment activated"

# Upgrade pip
print_step "Upgrading pip..."
pip install --upgrade pip
print_success "Pip upgraded"

# Install dependencies
print_step "Installing dependencies..."
pip install -r requirements.txt
if [ $? -ne 0 ]; then
    print_error "Some dependencies failed to install"
    echo "Trying minimal requirements..."
    pip install -r requirements_minimal.txt
    print_success "Minimal dependencies installed"
else
    print_success "All dependencies installed"
fi

# Create necessary directories
print_step "Creating necessary directories..."
mkdir -p config
mkdir -p models/yamnet_model
mkdir -p logs
mkdir -p src/{audio,speech,notification,models,hardware,gui,utils}
print_success "Directories created"

# Fix Python module structure
print_step "Fixing Python module structure..."
python fix_structure.py
python fix_imports.py
python create_init_files.py
print_success "Python module structure fixed"

# Fix configuration files
print_step "Creating configuration files..."
python create_config_files.py
print_success "Configuration files created"

# Download YAMNet model
print_step "Checking for YAMNet model..."
if [ ! -f "models/yamnet_model/yamnet.tflite" ]; then
    print_step "Downloading YAMNet model..."
    wget https://storage.googleapis.com/download.tensorflow.org/models/tflite/yamnet/yamnet.tflite -O models/yamnet_model/yamnet.tflite
    wget https://storage.googleapis.com/download.tensorflow.org/models/tflite/yamnet/yamnet_label_list.txt -O models/yamnet_model/yamnet_labels.txt
    if [ $? -ne 0 ]; then
        print_error "Failed to download YAMNet model. Using fallback mode."
        touch models/yamnet_model/yamnet.tflite
        touch models/yamnet_model/yamnet_labels.txt
    else
        print_success "YAMNet model downloaded"
    fi
else
    print_success "YAMNet model already exists"
fi

# Run verification tests
print_step "Running verification tests..."
python test_system.py --config
print_success "SenseBridge setup complete!"

echo -e "\n\033[1;34mNext steps:\033[0m"
echo "1. Run comprehensive tests: python test_system.py --all"
echo "2. Start SenseBridge: python -m src.main"
echo "   - For headless mode: python -m src.main --headless"
echo "   - For simulation mode: python -m src.main --simulation"
echo ""
echo "Remember to activate the environment with:"
echo "source .venv/bin/activate"