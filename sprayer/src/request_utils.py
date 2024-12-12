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