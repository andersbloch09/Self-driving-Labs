import os
import requests
from dotenv import load_dotenv


def main():
    # Load .env file from src directory (two directories up from script)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    env_path = os.path.join(script_dir, '..', '..', '.env')
    env = load_dotenv(env_path)
    # Replace with your actual Supabase API URL and API key
    # Try different possible URLs for Docker networking
    supabase_url = os.getenv('SUPABASE_URL', 'http://172.17.0.1:8000')  # Docker bridge network gateway (Linux)
    supabase_key = os.getenv('SERVICE_ROLE_KEY')  # Replace with your API key variable name
    function_name = 'hello'  # Replace with your table name
 
    api_url = supabase_url
    api_key = supabase_key

    try:
        # Define headers with the API key
        headers = {
            'apikey': api_key,
            'Authorization': f'Bearer {api_key}',
            'Content-Type': 'application/json'
        }
        
        # Construct the endpoint URL for the table
        endpoint_url = f"{api_url}/functions/v1/{function_name}"
        
        # Send a GET request to the REST API endpoint with headers
        response = requests.get(endpoint_url, headers=headers)
        
        # Check if the request was successful
        if response.status_code == 200:
            # Parse and print the JSON response
            data = response.json()
            print("Response from Supabase function API:")
            print(data)
        else:
            print(f"Failed to fetch data. HTTP Status Code: {response.status_code}")
            print("Response:", response.text)
    except Exception as e:
        print(f"Error: {e}")


    if __name__ == "__main__":
        main()
