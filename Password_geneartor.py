import base64
import hashlib
import secrets
# your password here
password = "user2@123" 
salt = secrets.token_bytes(16)
iterations = 310000

dk = hashlib.pbkdf2_hmac(
    "sha256",
    password.encode(),
    salt,
    iterations
)

salt_b64 = base64.urlsafe_b64encode(salt).decode()
hash_b64 = base64.urlsafe_b64encode(dk).decode()

print(f"pbkdf2_sha256${iterations}${salt_b64}${hash_b64}")