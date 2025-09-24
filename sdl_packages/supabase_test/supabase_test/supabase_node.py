import requests

url = "http://host.docker.internal:8000/rest/v1/your_table"
headers = {
    "apikey": "your-anon-key",
    "Authorization": "Bearer your-service-role-key"
}

response = requests.get(url, headers=headers)
if response.ok:
    data = response.json()
    print("Supabase data:", data)
else:
    print("Error:", response.status_code, response.text)
