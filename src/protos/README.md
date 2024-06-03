# Install protobuf for Python

```bash
pip install protobuf
```

# Generate schema for C++ and Python

```bash
protoc -I /dir/with/proto --cpp_out=/dir/generated --python_out=/dir/generated /path/to/schema.proto
```

# Compile C++ code

```bash
g++ myfile.cpp /path/to/schema.cc -lprotobuf -o myfile
```

# Check which protobuf implementation is used in Python

```bash
python -c "from google.protobuf.internal import api_implementation; print(api_implementation.Type())"
```

output: ```cpp``` or ```python```