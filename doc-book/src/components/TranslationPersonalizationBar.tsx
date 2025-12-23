// Translation and Personalization Component for Docusaurus
// File: doc-book/src/components/TranslationPersonalizationBar.tsx

import React, { useState, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';

interface PersonalizationPreferences {
  technicalDepth: 'beginner' | 'intermediate' | 'advanced';
  preferredExamples: string[];
  learningGoals: string[];
}

const TranslationPersonalizationBar: React.FC = () => {
  const [isUrdu, setIsUrdu] = useState(false);
  const [preferences, setPreferences] = useState<PersonalizationPreferences | null>(null);
  const location = useLocation();

  // Load preferences from localStorage when component mounts
  useEffect(() => {
    const savedPreferences = localStorage.getItem(`preferences-${location.pathname}`);
    if (savedPreferences) {
      setPreferences(JSON.parse(savedPreferences));
    }
  }, [location.pathname]);

  const handleTranslateToUrdu = () => {
    setIsUrdu(!isUrdu);
    // In a real implementation, this would call a translation service
    console.log(`Translation to Urdu ${isUrdu ? 'disabled' : 'enabled'}`);
  };

  const handlePersonalize = () => {
    // In a real implementation, this would open a personalization modal
    const newPreferences: PersonalizationPreferences = {
      technicalDepth: preferences?.technicalDepth || 'intermediate',
      preferredExamples: preferences?.preferredExamples || ['python'],
      learningGoals: preferences?.learningGoals || ['ros_fundamentals']
    };

    setPreferences(newPreferences);
    localStorage.setItem(`preferences-${location.pathname}`, JSON.stringify(newPreferences));

    console.log('Personalization preferences updated:', newPreferences);
  };

  return (
    <div className="translation-personalization-bar" style={{
      display: 'flex',
      justifyContent: 'flex-end',
      alignItems: 'center',
      padding: '10px',
      marginBottom: '20px',
      backgroundColor: '#f5f5f5',
      borderRadius: '4px'
    }}>
      <button
        className="translate-button"
        onClick={handleTranslateToUrdu}
        style={{
          marginRight: '10px',
          padding: '5px 10px',
          backgroundColor: isUrdu ? '#d3b323ff' : '#2196F3',
          color: 'white',
          border: 'none',
          borderRadius: '4px',
          cursor: 'pointer'
        }}
      >
        {isUrdu ? 'Switch to English' : '.Translate to Urdu'}
      </button>
      <button
        className="personalize-button"
        onClick={handlePersonalize}
        style={{
          padding: '5px 10px',
          backgroundColor: '#00ffddff',
          color: 'white',
          border: 'none',
          borderRadius: '4px',
          cursor: 'pointer'
        }}
      >
        Personalize
      </button>
    </div>
  );
};

export default TranslationPersonalizationBar;