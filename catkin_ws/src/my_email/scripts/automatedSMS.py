import http.client
import json

print('hello')
conn = http.client.HTTPSConnection("6g2xwr.api.infobip.com")
payload = json.dumps({
    "messages": [
        {
            "destinations": [{"to":"447767201189"}],
            "from": "ServiceSMS",
            "text": "Congratulations on sending your first message.\nGo ahead and check the delivery report in the next step."
        }
    ]
})
headers = {
    'Authorization': 'App 11fa5f4e2e98c4873093c456fdfc16dd-a4edb884-fc3e-480f-a4cc-644a0551f58a',
    'Content-Type': 'application/json',
    'Accept': 'application/json'
}
conn.request("POST", "/sms/2/text/advanced", payload, headers)
res = conn.getresponse()
data = res.read()
print(data.decode("utf-8"))