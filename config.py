import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

class Config:
    """Configuration class to manage API keys and other settings"""
    
    GEMINI_API_KEY = os.getenv('GEMINI_API_KEY')
    
    @classmethod
    def validate(cls):
        """Validate that all required configuration is present"""
        if not cls.GEMINI_API_KEY:
            raise ValueError("GEMINI_API_KEY is not set in .env file")
        
    @classmethod
    def get_api_key(cls):
        """Safely get the API key"""
        cls.validate()
        return cls.GEMINI_API_KEY 