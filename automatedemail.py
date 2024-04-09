import smtplib
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from datetime import datetime
import pytz

def send_email():
    # Email details

    sender_email = 'eliott.laurent@ecam.fr'
    sender_password = 'Eliott2907?'

    recipient_email = 'eliott.laurent1@gmail.com'
    # Get current time in UK timezone
    uk_timezone = pytz.timezone('Europe/London')
    current_time = datetime.now(uk_timezone).strftime('%Y-%m-%dT%H:%M:%S%z')
    subject = 'Automated Email'
    message = f'Hello, this is an automated email sent using Python!\nTimestamp:{current_time}'

    # Create message container
    msg = MIMEMultipart()
    msg['From'] = sender_email
    msg['To'] = recipient_email
    msg['Subject'] = subject

    # Attach message
    msg.attach(MIMEText(message, 'plain'))

    # Connect to SMTP server (Gmail)
    server = smtplib.SMTP('smtp.ecam.fr', 587)
    server.starttls()
    server.login(sender_email, sender_password)

    # Send email
    text = msg.as_string()
    server.sendmail(sender_email, recipient_email, text)

    # Close the SMTP server connection
    server.quit()

if __name__ == "__main__":
    send_email()
    print("Email sent successfully!")
