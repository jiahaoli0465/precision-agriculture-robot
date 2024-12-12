# Retry Request FAQ

## Background

Since our VNC environment has unstable internet connectivity, we implemented `request_utils.py` to handle HTTP request failures gracefully. This utility provides automatic retry functionality with exponential backoff to improve reliability of network operations.

## General Questions

### What does request_utils.py do?

The utility provides wrapper functions for making HTTP requests with built-in retry logic. If a request fails due to network issues, it will automatically retry the request with increasing delays between attempts.

### What are the default retry settings?

- Maximum retries: 3 attempts
- Base delay: 1 second
- Exponential backoff: Delay doubles after each retry
  - 1st retry: 1 second
  - 2nd retry: 2 seconds
  - 3rd retry: 4 seconds

### How do I use the utility functions?

```python
from request_utils import make_get_request, make_post_request

# GET request
response = make_get_request('http://example.com/api')

# POST request
response = make_post_request('http://example.com/api', {'data': 'value'})
```

## Common Issues and Solutions

### Q: The requests are timing out too quickly

By default, requests have a 1-second timeout. You can increase this:

```python
response = make_get_request(url, timeout=5.0)  # 5 second timeout
```

### Q: I need more retry attempts

Modify the decorator in request_utils.py:

```python
@retry_request(max_retries=5, base_delay=1)
def make_get_request(url, timeout=1.0):
    return requests.get(url, timeout=timeout)
```

### Q: How can I tell if retries are happening?

Watch the ROS logs. The utility logs each retry attempt with:

- The attempt number
- The error message
- The delay before the next retry

### Q: What happens after all retries fail?

- A warning is logged via `rospy.logwarn`
- The original exception is re-raised
- Your code should handle this exception appropriately

## Best Practices

1. **Always check responses**

```python
response = make_get_request(url)
if response and response.status_code == 200:
    # Process successful response
```

2. **Handle exceptions**

```python
try:
    response = make_get_request(url)
except requests.exceptions.RequestException as e:
    rospy.logwarn(f"Request failed after all retries: {e}")
    # Handle failure appropriately
```

3. **Adjust timeouts based on operation**

- Use shorter timeouts for frequent status checks
- Use longer timeouts for operations involving data transfer

## Logging

The utility logs the following events:

- Failed request attempts
- Retry delays
- Maximum retry limit reached
- Unexpected errors

Example log messages:

```
[WARN] Request failed (attempt 1/3): Connection timeout
[INFO] Retrying in 1 seconds...
[WARN] Request failed (attempt 2/3): Connection timeout
[INFO] Retrying in 2 seconds...
[WARN] Max retries (3) reached. Last error: Connection timeout
```

## Configuration Tips

### For Very Unstable Connections

```python
@retry_request(max_retries=5, base_delay=2)
def make_get_request(url, timeout=2.0):
    return requests.get(url, timeout=timeout)
```

### For Large Data Transfers

```python
@retry_request(max_retries=3, base_delay=2)
def make_post_request(url, json_data):
    return requests.post(url, json=json_data, timeout=5.0)
```

## full code:

````python
import requests
from functools import wraps
import time
import rospy

def retry_request(max_retries=3, base_delay=1):
    """
    Decorator that implements retry logic for HTTP requests with exponential backoff.

    Args:
        max_retries (int): Maximum number of retry attempts
        base_delay (int): Base delay between retries in seconds
    """
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            retries = 0
            while retries < max_retries:
                try:
                    return func(*args, **kwargs)
                except requests.exceptions.RequestException as e:
                    retries += 1
                    if retries == max_retries:
                        rospy.logwarn(f"Max retries ({max_retries}) reached. Last error: {e}")
                        raise

                    # Calculate delay with exponential backoff
                    delay = base_delay * (2 ** (retries - 1))
                    rospy.logwarn(f"Request failed (attempt {retries}/{max_retries}): {e}")
                    rospy.loginfo(f"Retrying in {delay} seconds...")
                    time.sleep(delay)
            return None
        return wrapper
    return decorator

@retry_request()
def make_get_request(url, timeout=1.0):
    """Make GET request with retry logic"""
    return requests.get(url, timeout=timeout)

@retry_request()
def make_post_request(url, json_data):
    """Make POST request with retry logic"""
    return requests.post(url, json=json_data)
    ```
````
